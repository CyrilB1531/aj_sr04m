/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "aj_sr04m.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aj_sr04m_priv.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_rom_sys.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"

#if AJ_SR04M_MODE >= 3
#include "driver/uart.h"
#endif

#define AJ_SR04M_TAG "AJ-SR04M"

#define AJ_SR04M_DIST_MIN_VALID_MM 200  /* sensor physical limit */
#define AJ_SR04M_DIST_MAX_VALID_MM 4500 /* sensor physical limit */

/* Binary frame (modes 3-4): header | dist_H | dist_L | checksum */
#define AJ_SR04M_BINARY_FRAME_LEN 4
#define AJ_SR04M_BINARY_FRAME_HEADER 0xFF

/* RMT capture parameters: shared by modes 1-2 (echo) and the modes 4-5
 * software UART backend (frame capture). */
#define AJ_SR04M_RMT_RESOLUTION_HZ 1000000U /* 1 MHz -> 1 us per RMT tick */
/* Capture capacity, in RMT symbols. Each symbol carries two level runs, so
 * 64 symbols hold 128 of them. The longest frame captured here is the mode 5
 * ASCII payload "Gap=XXXX mm\r\n": 13 bytes at 9600 8N1 is 130 line bits,
 * but identical neighbouring bits merge into one run, and over every digit
 * combination the worst case is 86 runs — 43 symbols. A binary frame of
 * modes 3-4 needs 15, an echo pulse of modes 1-2 needs two.
 *
 * 64 is also the ESP32's RMT memory block size, and its RX path has no
 * ping-pong support: a capture there cannot outgrow the blocks reserved when
 * the channel was created, so asking for more takes a second block away from
 * another channel. The margin above is what buys the fixed size; a capture
 * that reaches capacity is treated as truncated rather than decoded. */
#define AJ_SR04M_RMT_NUM_SYMBOLS 64

/* Idle threshold (rmt_receive_config_t::signal_range_max_ns). RMT ends a
 * capture as soon as *one* level run outlasts it — the echo pulse is such a
 * run, not only the silence that follows it — so the two capture kinds size
 * it from different worst cases and keep separate constants.
 *
 * Modes 1-2, single echo pulse. The floor is the longest pulse the
 * [200, 4500] mm window still accepts: 4500 mm / 0.1715 mm per us = 26.2 ms.
 * A shorter threshold would cut a far target's pulse in flight, and the
 * truncated run is reported like any other: a 6 m wall would come back as a
 * confident measurement worth the threshold itself instead of NO_ECHO. The
 * ceiling is the 15-bit RMT duration counter, 32767 ticks at 1 MHz = 32.7 ms.
 * 30 ms sits inside [26.3, 32.7] with margin on both sides, so every pulse
 * longer than a valid echo is still measured, still lands above 4500 mm and
 * is still rejected on its value. */
#define AJ_SR04M_RMT_ECHO_IDLE_NS 30000000U

/* Modes 4-5 software backend, UART frame. Nothing here ends the capture but
 * the line going idle after the last stop bit, so the threshold delimits the
 * frame and has to outlast any gap the module leaves *between* two bytes of
 * one reply. 30 ms is far above the 1.04 ms of a 9600 baud character, and it
 * costs latency the reply timeout below already dominates. */
#define AJ_SR04M_RMT_FRAME_IDLE_NS 30000000U

/* Modes 1-2 read timeout. The read does not wait for the echo's falling edge
 * but for RMT to declare the capture over, one idle threshold later:
 *
 *   echo pulse         <= 30 ms  (a longer level ends the capture on its own)
 * + idle threshold        30 ms
 * + 40 ms  for the trigger pulse (1.1 ms in mode 2), the module's
 *          burst-to-echo latency, and the FreeRTOS tick quantisation —
 *          pdMS_TO_TICKS() floors to the tick period and the wait may end one
 *          tick short, i.e. up to 20 ms at the default 100 Hz.
 *
 * The 50 ms this used to be was sized on the round trip alone, which is not
 * what the read waits for: past ~3.4 m the timeout expired while the
 * measurement was already sitting in the buffer, and the miss surfaced as
 * NO_ECHO — a sensor fault rather than a driver one. */
#define AJ_SR04M_RMT_TIMEOUT_MS                                                \
  (2 * (AJ_SR04M_RMT_ECHO_IDLE_NS / 1000000U) + 40)

/* UART modes (3-5) wait on the module, not on an echo: a reply lands
 * ~100-200 ms after the trigger byte, so the echo budget above — sized for a
 * 4.5 m round trip and its idle threshold — expires long before it. */
#define AJ_SR04M_UART_REPLY_TIMEOUT_MS 250

/* Mode 3 is not prompted: nothing the driver does starts a measurement, so
 * the read waits for the next frame of a free-running stream instead of for
 * a reply. The module pushes one about every 120 ms — a cadence observed on
 * the bench, not stated in the Mantech datasheet, like the mode 5 ASCII
 * payload. A window narrower than that period lands between two frames more
 * often than not, and the empty buffer that comes back is indistinguishable
 * here from a malformed one: a healthy sensor is reported as BAD_FRAME.
 *
 * One whole stream period, plus 30 ms for the module's own jitter, plus
 * 20 ms for the FreeRTOS tick quantisation — pdMS_TO_TICKS() floors to the
 * tick period and the wait may still end one tick short, 10 ms each at the
 * default 100 Hz. */
#define AJ_SR04M_UART_STREAM_TIMEOUT_MS (120 + 30 + 20)

/* Modes 3 and 4 share one read call site on the hardware backend, but not
 * what they wait on: mode 4 is prompted and waits out the module's reply
 * latency, mode 3 waits out its stream period. */
#if AJ_SR04M_MODE == 3
#define AJ_SR04M_UART_BINARY_READ_TIMEOUT_MS AJ_SR04M_UART_STREAM_TIMEOUT_MS
#else
#define AJ_SR04M_UART_BINARY_READ_TIMEOUT_MS AJ_SR04M_UART_REPLY_TIMEOUT_MS
#endif

/* The software backend waits for the same reply, then for the line to sit
 * idle for AJ_SR04M_RMT_FRAME_IDLE_NS before RMT reports the capture
 * complete. */
#define AJ_SR04M_SW_UART_CAPTURE_TIMEOUT_MS                                    \
  (AJ_SR04M_UART_REPLY_TIMEOUT_MS + (AJ_SR04M_RMT_FRAME_IDLE_NS / 1000000U) +  \
   20)

#define AJ_SR04M_MAX_SENSORS CONFIG_AJ_SR04M_MAX_SENSORS

/* Global state: array of sensor instances */
static aj_sr04m_sensor_t s_sensors[AJ_SR04M_MAX_SENSORS];
static aj_sr04m_handle_t s_sensor_handles[AJ_SR04M_MAX_SENSORS];
static int s_sensor_count = 0;
static bool s_initialized = false;

/* Guards every read and write of the four variables above. Without it, a
 * task deleting a sensor shifts s_sensor_handles under a task walking it in
 * aj_sr04m_trigger_all() — and that walk yields on the trigger stagger, so
 * the window is milliseconds wide, not instructions wide. */
static SemaphoreHandle_t s_table_mutex;
static StaticSemaphore_t s_table_mutex_storage;

/* The mutex is built by a constructor, out of static storage, for three
 * reasons a create-on-first-use scheme cannot satisfy:
 *
 *  - it exists before the first caller. Testing the handle for NULL and
 *    creating it when unset is itself the race it is meant to close, and no
 *    entry point of this driver is guaranteed to run first.
 *  - it cannot fail, so no caller has to cope with a table that has no lock.
 *    xSemaphoreCreateMutexStatic() only ever hands back a handle onto the
 *    buffer above; there is no allocation to run out of.
 *  - it outlives aj_sr04m_deinit(). Destroying it there and recreating it in
 *    aj_sr04m_init() would move the race rather than remove it: that
 *    destruction would itself need serialising against the callers it races.
 *
 * Constructors run single-threaded before any task exists: on ESP targets
 * from do_global_ctors(), after the heap is up and before the scheduler
 * starts; on the linux port before main(). */
static void __attribute__((constructor)) aj_sr04m_table_mutex_init(void) {
  s_table_mutex = xSemaphoreCreateMutexStatic(&s_table_mutex_storage);
}

/* Blocks until the table is ours. No entry point of this driver is callable
 * from an interrupt handler, so an indefinite wait is the right one: the
 * only holders are other driver calls, which all release it. */
static void aj_sr04m_table_lock(void) {
  xSemaphoreTake(s_table_mutex, portMAX_DELAY);
}

static void aj_sr04m_table_unlock(void) { xSemaphoreGive(s_table_mutex); }

/* RMT RX done callback: shared by modes 1-2 echo capture and the modes 4-5
 * software UART backend. The signature is imposed by ESP-IDF's
 * rmt_rx_done_callback_t; the channel is not needed, since the sensor
 * instance arrives through user_data.
 *
 * NOSONAR on the parameter line: c:S995 asks for a pointer-to-const
 * channel, which would change the function type and make it incompatible
 * with the rmt_rx_event_callbacks_t field this is assigned to. */
static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel, // NOSONAR
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_data) {
  (void)channel;

  BaseType_t hp_task_woken = pdFALSE;
  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)user_data;
  sensor->rx_num_symbols = edata->num_symbols;
  xSemaphoreGiveFromISR(sensor->rx_done_sem, &hp_task_woken);
  return hp_task_woken == pdTRUE;
}

/* Releases whichever RMT capture resources the sensor currently owns and
 * clears the pointers, so a slot left behind by a failed aj_sr04m_new()
 * carries no dangling handle. Safe to call at any point of the setup
 * sequence: every field is checked before being released.
 *
 * The order is the reverse of the acquisition, and that is what makes it
 * safe rather than merely tidy. The channel goes first because rmt_disable()
 * is the point at which ESP-IDF stops delivering completions: an armed
 * capture finishing after the semaphore had been deleted would enter
 * rmt_rx_done_cb() in interrupt context and give a freed handle. Everything
 * the callback touches — rx_done_sem, and the sensor slot itself, which
 * holds the rx_num_symbols it writes — therefore has to outlive the channel.
 *
 * @param sensor         sensor slot to release
 * @param disable_channel true once rmt_enable() has succeeded */
static void aj_sr04m_release_rmt_resources(aj_sr04m_sensor_t *sensor,
                                           bool disable_channel) {
  if (sensor->rx_channel != NULL) {
    /* A channel that was never enabled delivers nothing and would refuse
     * rmt_disable(), so the setup exits reached before rmt_enable()
     * succeeded skip this step. */
    if (disable_channel) {
      rmt_disable(sensor->rx_channel);
    }
    rmt_del_channel(sensor->rx_channel);
    sensor->rx_channel = NULL;
  }
  if (sensor->rx_done_sem != NULL) {
    vSemaphoreDelete(sensor->rx_done_sem);
    sensor->rx_done_sem = NULL;
  }
  if (sensor->rx_buffer != NULL) {
    free(sensor->rx_buffer);
    sensor->rx_buffer = NULL;
  }
}

/* Arms the RMT receiver for one capture window.
 *
 * The drain is what makes a cycle independent of the previous one. A capture
 * that completes after its read gave up leaves rx_done_sem signalled; without
 * clearing it here, the next read would take that stale give immediately and
 * decode rx_buffer while RMT is concurrently writing the new capture into it.
 * The result is a torn buffer read with an rx_num_symbols belonging to
 * neither capture — and the parsers may well accept it. */
static esp_err_t aj_sr04m_arm_rmt_capture(aj_sr04m_sensor_t *sensor) {
  xSemaphoreTake(sensor->rx_done_sem, 0);

  rmt_receive_config_t rx_cfg = {
    .signal_range_min_ns = 1000, /* filter glitches < 1 us */
  /* Idle threshold, i.e. what ends this capture. The mode is a
   * compile-time choice and the two capture kinds never coexist, so the
   * arm picks the threshold sized for the one being armed. */
#if AJ_SR04M_MODE <= 2
    .signal_range_max_ns = AJ_SR04M_RMT_ECHO_IDLE_NS,
#else
    .signal_range_max_ns = AJ_SR04M_RMT_FRAME_IDLE_NS,
#endif
  };
  /* A failed arm leaves the receiver idle, so the read that follows would
   * time out and report NO_ECHO — the same status as a sensor pointing at
   * open air. Reporting it keeps a driver fault from passing as a plausible
   * measurement outcome. */
  esp_err_t err = rmt_receive(
      sensor->rx_channel, sensor->rx_buffer,
      AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t), &rx_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to arm RMT capture on echo pin %d: %s",
             sensor->echo_pin, esp_err_to_name(err));
  }
  return err;
}

/* Returns the TRIG (modes 1-2) / software UART TX pin to a high-impedance
 * input, so a deleted sensor stops driving the line. */
static void aj_sr04m_release_trigger_pin(int trigger_pin) {
  gpio_config_t idle_cfg = {
      .pin_bit_mask = 1ULL << trigger_pin,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&idle_cfg);
}

#if AJ_SR04M_MODE >= 3
/* Unwinds a hardware UART backend that could not be brought up, leaving the
 * TX pin with nothing driving it.
 *
 * Deleting the driver is not enough on its own. ESP-IDF releases the pins
 * from there, but that release only clears the pin's GPIO output-enable bit,
 * which a pad routed through the IOMUX ignores — and the IOMUX is the route
 * uart_set_pin() takes whenever the requested pin is the chip's dedicated
 * pad for that signal, as GPIO 17 is for UART2 TX on the ESP32, this
 * component's default TRIG. gpio_config() additionally resets the pad's
 * function select back to plain GPIO, which is what actually detaches the
 * peripheral. uart_set_pin() with UART_PIN_NO_CHANGE could not do it either:
 * that value means "leave this one alone", not "give it back".
 *
 * The order is load-bearing. uart_driver_delete() reconfigures the very pins
 * being parked, so parking has to come last or the teardown writes over it.
 *
 * Only TX is parked. RX is an input: any routing that survives feeds the
 * deleted peripheral's receive signal and drives nothing on the board, while
 * rewriting that pad would change state this failure may never have created.
 *
 * @param uart_num    port whose driver is installed
 * @param trigger_pin pin handed to uart_set_pin() as TX */
static void aj_sr04m_release_hw_uart(uart_port_t uart_num, int trigger_pin) {
  uart_driver_delete(uart_num);
  aj_sr04m_release_trigger_pin(trigger_pin);
}
#endif

/* A capture that fills the buffer was cut short. The RMT engine stops at the
 * end of the memory it was given, logs from its ISR, and still reports the
 * symbols it managed to store — so the tail of the frame is simply missing,
 * and what remains decodes into a plausible-looking but wrong measurement.
 * An exactly-full capture is indistinguishable from a truncated one, hence
 * the margin the buffer is sized with: reaching capacity means something is
 * wrong (a floating echo pin oscillating, a module streaming without an idle
 * gap), not that a frame happened to fit. */
static bool aj_sr04m_capture_truncated(const aj_sr04m_sensor_t *sensor) {
  if (sensor->rx_num_symbols < AJ_SR04M_RMT_NUM_SYMBOLS)
    return false;

  ESP_LOGE(AJ_SR04M_TAG,
           "Capture truncated on echo pin %d: %u symbols fill the buffer",
           sensor->echo_pin, (unsigned)sensor->rx_num_symbols);
  return true;
}

#if AJ_SR04M_MODE < 3
static uint32_t extract_high_pulse_us(const aj_sr04m_sensor_t *sensor) {
  for (size_t i = 0; i < sensor->rx_num_symbols; i++) {
    const rmt_symbol_word_t *sym = &sensor->rx_buffer[i];
    if (sym->level0 == 1 && sym->duration0 > 0)
      return sym->duration0;
    if (sym->level1 == 1 && sym->duration1 > 0)
      return sym->duration1;
  }
  return 0;
}
#endif

#if AJ_SR04M_MODE >= 3
#define AJ_SR04M_BUFFER_SIZE 128
#define AJ_SR04M_SENSOR_1_UART_PORT CONFIG_AJ_SR04M_UART_NUM
#define AJ_SR04M_SENSOR_2_UART_PORT CONFIG_AJ_SR04M_SENSOR_2_UART_NUM
#define AJ_SR04M_SENSOR_3_UART_PORT CONFIG_AJ_SR04M_SENSOR_3_UART_NUM
#define AJ_SR04M_SENSOR_4_UART_PORT CONFIG_AJ_SR04M_SENSOR_4_UART_NUM
#else
/* GPIO modes (1-2) ignore the UART port; these placeholders keep the
 * shared aj_sr04m_new() signature usable without the UART Kconfig
 * symbols (which only exist in modes 3-5). */
#define AJ_SR04M_SENSOR_1_UART_PORT 0
#define AJ_SR04M_SENSOR_2_UART_PORT 0
#define AJ_SR04M_SENSOR_3_UART_PORT 0
#define AJ_SR04M_SENSOR_4_UART_PORT 0
#endif

aj_sr04m_dist_status_t aj_sr04m_parse_binary_frame(const uint8_t *data, int len,
                                                   int16_t *distance) {
  if (len != 4 || data[0] != 0xFF)
    return AJ_SR04M_DIST_BAD_FRAME;

  uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;
  if (checksum != data[3])
    return AJ_SR04M_DIST_BAD_CHECKSUM;

  int16_t mm = ((int16_t)data[1] << 8) | (int16_t)data[2];
  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = mm;
  return AJ_SR04M_DIST_OK;
}

aj_sr04m_dist_status_t
aj_sr04m_parse_binary_stream(const uint8_t *data, int len, int16_t *distance) {
  if (data == NULL || len < AJ_SR04M_BINARY_FRAME_LEN)
    return AJ_SR04M_DIST_BAD_FRAME;

  /* Backwards, so the first frame accepted is the newest one in the buffer.
   * A distance field can hold 0xFF, so a header match alone does not make a
   * frame: only a matching checksum ends the scan. */
  bool saw_bad_checksum = false;
  for (int i = len - AJ_SR04M_BINARY_FRAME_LEN; i >= 0; i--) {
    if (data[i] != AJ_SR04M_BINARY_FRAME_HEADER)
      continue;

    aj_sr04m_dist_status_t status = aj_sr04m_parse_binary_frame(
        &data[i], AJ_SR04M_BINARY_FRAME_LEN, distance);
    if (status == AJ_SR04M_DIST_BAD_CHECKSUM) {
      saw_bad_checksum = true;
      continue;
    }
    return status;
  }

  /* Headers were present but none checksummed: the bytes are frames, damaged
   * in transit. Reporting BAD_FRAME instead would hide that distinction. */
  return saw_bad_checksum ? AJ_SR04M_DIST_BAD_CHECKSUM
                          : AJ_SR04M_DIST_BAD_FRAME;
}

aj_sr04m_dist_status_t aj_sr04m_parse_ascii_frame(const char *data,
                                                  int16_t *distance) {
  if (data == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  const char *p = strstr(data, "Gap=");
  if (p == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  unsigned int mm = 0;
  if (sscanf(p, "Gap=%u mm", &mm) != 1)
    return AJ_SR04M_DIST_BAD_FRAME;

  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = (int16_t)mm;
  return AJ_SR04M_DIST_OK;
}

/* Everything from here to aj_sr04m_read_all_locked() touches the sensor
 * table and must run with s_table_mutex held. The public entry points at the
 * bottom of the file are thin wrappers that take the lock, call the matching
 * _locked helper and release it — which is also what keeps the many early
 * `return`s of aj_sr04m_new_locked() from having to remember to unlock. */
static void aj_sr04m_register_handle(aj_sr04m_handle_t handle) {
  if (s_sensor_count < AJ_SR04M_MAX_SENSORS) {
    s_sensor_handles[s_sensor_count++] = handle;
  }
}

/* s_sensor_count never exceeds AJ_SR04M_MAX_SENSORS: only
 * aj_sr04m_register_handle() grows it, under that bound. Every loop below
 * still repeats the bound explicitly, so the limit holds locally instead of
 * resting on an invariant established in another function. */
static int aj_sr04m_registered_count(void) {
  return s_sensor_count < AJ_SR04M_MAX_SENSORS ? s_sensor_count
                                               : AJ_SR04M_MAX_SENSORS;
}

static void aj_sr04m_unregister_handle(const struct aj_sr04m_sensor *handle) {
  const int count = aj_sr04m_registered_count();

  int index = -1;
  for (int i = 0; i < count; i++) {
    if (s_sensor_handles[i] == handle) {
      index = i;
      break;
    }
  }

  if (index < 0)
    return;

  for (int i = index; i + 1 < count; i++) {
    s_sensor_handles[i] = s_sensor_handles[i + 1];
  }
  s_sensor_handles[count - 1] = NULL;
  s_sensor_count--;
}

static aj_sr04m_handle_t aj_sr04m_new_locked(int trigger_pin, int echo_pin,
                                             uint8_t trigger_byte,
                                             int uart_num);
static void aj_sr04m_delete_locked(aj_sr04m_handle_t handle);

static void aj_sr04m_cleanup_configured_sensors(void) {
  while (s_sensor_count > 0) {
    aj_sr04m_delete_locked(s_sensor_handles[0]);
  }
}

static esp_err_t aj_sr04m_configure_sensors_from_kconfig(void) {
  const struct aj_sr04m_sensor *handle = aj_sr04m_new_locked(
      CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_1_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;

#if AJ_SR04M_MAX_SENSORS >= 2
  handle = aj_sr04m_new_locked(
      CONFIG_AJ_SR04M_SENSOR_2_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_2_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_2_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

#if AJ_SR04M_MAX_SENSORS >= 3
  handle = aj_sr04m_new_locked(
      CONFIG_AJ_SR04M_SENSOR_3_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_3_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_3_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

#if AJ_SR04M_MAX_SENSORS >= 4
  handle = aj_sr04m_new_locked(
      CONFIG_AJ_SR04M_SENSOR_4_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_4_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_4_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

  return ESP_OK;
}

esp_err_t aj_sr04m_init(void) {
  aj_sr04m_table_lock();

  /* Single exit, so the unlock cannot be skipped by a path added later. */
  esp_err_t err = ESP_OK;
  if (!s_initialized) {
    memset(s_sensors, 0, sizeof(s_sensors));
    memset(s_sensor_handles, 0, sizeof(s_sensor_handles));
    s_sensor_count = 0;
    s_initialized = true;

    err = aj_sr04m_configure_sensors_from_kconfig();
    if (err != ESP_OK) {
      aj_sr04m_cleanup_configured_sensors();
      s_initialized = false;
    }

#if AJ_SR04M_MODE >= 3
    /* UART modes: global driver setup if needed (per-sensor UART config
     * happens in aj_sr04m_new) */
#endif
  }

  aj_sr04m_table_unlock();
  return err;
}

esp_err_t aj_sr04m_deinit(void) {
  aj_sr04m_table_lock();

  aj_sr04m_cleanup_configured_sensors();
  memset(s_sensors, 0, sizeof(s_sensors));
  memset(s_sensor_handles, 0, sizeof(s_sensor_handles));
  s_sensor_count = 0;
  s_initialized = false;

  aj_sr04m_table_unlock();
  return ESP_OK;
}

/* Single unwind for every setup step of aj_sr04m_new() that owns the TRIG /
 * software UART TX pin. Splitting it per exit is how the two branches came to
 * release opposite halves of what they had taken: modes 1-2 handed the pin
 * back on the early exits only, the software backend on the RMT ones only,
 * and a sensor that failed to build kept driving the line either way — low in
 * modes 1-2, high on the software backend, which is the UART idle level a
 * module reads as a peer that is still there.
 *
 * Both releases tolerate being reached before the matching acquisition:
 * aj_sr04m_release_rmt_resources() skips the fields still NULL, and
 * reconfiguring an untouched pin as a high-impedance input costs nothing.
 * That is what lets even the gpio_config() failure come through here — that
 * call applies the pin one setting at a time, so it can fail with the pin
 * already an output, and the exit cannot tell which. The hardware UART
 * backend never calls this: its pins belong to the UART peripheral, and
 * uart_driver_delete() is what hands them back. */
static void aj_sr04m_release_partial_sensor(aj_sr04m_sensor_t *sensor) {
  aj_sr04m_release_rmt_resources(sensor, false);
  aj_sr04m_release_trigger_pin(sensor->trigger_pin);
}

static aj_sr04m_handle_t aj_sr04m_new_locked(int trigger_pin, int echo_pin,
                                             uint8_t trigger_byte,
                                             int uart_num) {
  if (!s_initialized) {
    ESP_LOGE(AJ_SR04M_TAG,
             "Driver not initialized. Call aj_sr04m_init() first.");
    return NULL;
  }

  /* Find a free slot */
  aj_sr04m_sensor_t *sensor = NULL;
  for (int i = 0; i < AJ_SR04M_MAX_SENSORS; i++) {
    if (!s_sensors[i].initialized) {
      sensor = &s_sensors[i];
      break;
    }
  }

  if (sensor == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Max sensors (%d) reached", AJ_SR04M_MAX_SENSORS);
    return NULL;
  }

  /* Initialize the sensor structure */
  sensor->trigger_pin = trigger_pin;
  sensor->echo_pin = echo_pin;
  sensor->trigger_byte = trigger_byte;

#if AJ_SR04M_MODE < 3
  /* GPIO + RMT mode setup */
  gpio_config_t trigger_cfg = {
      .pin_bit_mask = 1ULL << trigger_pin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  if (gpio_config(&trigger_cfg) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to configure trigger pin %d", trigger_pin);
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  if (gpio_set_level(trigger_pin, 0) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to set trigger pin %d low", trigger_pin);
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  /* Allocate RMT RX buffer */
  sensor->rx_buffer =
      malloc(AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t));
  if (sensor->rx_buffer == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate RMT RX buffer");
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  /* Configure and allocate RMT RX channel */
  rmt_rx_channel_config_t rx_chan_cfg = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = AJ_SR04M_RMT_RESOLUTION_HZ,
      .mem_block_symbols = AJ_SR04M_RMT_NUM_SYMBOLS,
      .gpio_num = echo_pin,
  };
  if (rmt_new_rx_channel(&rx_chan_cfg, &sensor->rx_channel) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate RMT RX channel for pin %d",
             echo_pin);
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  /* Create semaphore for RMT done callback */
  sensor->rx_done_sem = xSemaphoreCreateBinary();
  if (sensor->rx_done_sem == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to create RMT semaphore");
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  /* Register RMT callback */
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_cb,
  };
  if (rmt_rx_register_event_callbacks(sensor->rx_channel, &cbs, sensor) !=
      ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to register RMT callback");
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

  /* Enable RMT channel */
  if (rmt_enable(sensor->rx_channel) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to enable RMT channel");
    aj_sr04m_release_partial_sensor(sensor);
    return NULL;
  }

#else
  /* UART mode setup: resolve hardware vs software (RMT) backend. */
  sensor->backend = aj_sr04m_sw_uart_resolve_backend(uart_num, SOC_UART_NUM);

  if (sensor->backend == AJ_SR04M_UART_BACKEND_HW) {
    const int uart_buffer_size = (AJ_SR04M_BUFFER_SIZE * 2);
    sensor->uart_num = (uart_port_t)uart_num;

    if (uart_driver_install(sensor->uart_num, uart_buffer_size, 0, 10, NULL,
                            0) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to install UART driver on port %d",
               sensor->uart_num);
      return NULL;
    }

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(sensor->uart_num, &uart_config) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to configure UART %d", sensor->uart_num);
      /* Baud rate, frame format and clock source only, all of them register
       * writes on the peripheral: pin routing lives entirely in
       * uart_set_pin(), which has not run yet. Nothing to give back beyond
       * the driver itself. */
      uart_driver_delete(sensor->uart_num);
      return NULL;
    }

    if (uart_set_pin(sensor->uart_num, trigger_pin, echo_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to set UART pins");
      /* Routing is applied signal by signal, TX first, so a failure here can
       * still leave the trigger pin wired to the peripheral. Handing back a
       * NULL sensor whose pin keeps driving the line is what this releases. */
      aj_sr04m_release_hw_uart(sensor->uart_num, trigger_pin);
      return NULL;
    }
    ESP_LOGI(AJ_SR04M_TAG, "Sensor UART(HW port %d): tx=%d rx=%d",
             sensor->uart_num, trigger_pin, echo_pin);
  } else {
    /* Software UART: GPIO TX (idle high) + RMT RX on the echo pin. */
    gpio_config_t tx_cfg = {
        .pin_bit_mask = 1ULL << trigger_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&tx_cfg) != ESP_OK ||
        gpio_set_level(trigger_pin, 1) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to configure SW UART TX pin %d",
               trigger_pin);
      aj_sr04m_release_partial_sensor(sensor);
      return NULL;
    }

    sensor->rx_buffer =
        malloc(AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t));
    if (sensor->rx_buffer == NULL) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate SW UART RX buffer");
      aj_sr04m_release_partial_sensor(sensor);
      return NULL;
    }

    rmt_rx_channel_config_t rx_chan_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = AJ_SR04M_RMT_RESOLUTION_HZ,
        .mem_block_symbols = AJ_SR04M_RMT_NUM_SYMBOLS,
        .gpio_num = echo_pin,
    };
    if (rmt_new_rx_channel(&rx_chan_cfg, &sensor->rx_channel) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate SW UART RMT channel");
      aj_sr04m_release_partial_sensor(sensor);
      return NULL;
    }

    sensor->rx_done_sem = xSemaphoreCreateBinary();
    if (sensor->rx_done_sem == NULL) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to create SW UART RMT semaphore");
      aj_sr04m_release_partial_sensor(sensor);
      return NULL;
    }

    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rmt_rx_done_cb};
    if (rmt_rx_register_event_callbacks(sensor->rx_channel, &cbs, sensor) !=
            ESP_OK ||
        rmt_enable(sensor->rx_channel) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to start SW UART RMT capture");
      aj_sr04m_release_partial_sensor(sensor);
      return NULL;
    }
    ESP_LOGI(AJ_SR04M_TAG, "Sensor UART(SW/RMT): tx=%d rx=%d", trigger_pin,
             echo_pin);
  }

#endif

  sensor->initialized = true;
  aj_sr04m_register_handle((aj_sr04m_handle_t)sensor);
  ESP_LOGI(AJ_SR04M_TAG, "Sensor created: trigger=%d, echo=%d", trigger_pin,
           echo_pin);

  return (aj_sr04m_handle_t)sensor;
}

static void aj_sr04m_delete_locked(aj_sr04m_handle_t handle) {
  if (handle == NULL)
    return;

  aj_sr04m_unregister_handle(handle);

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return;

#if AJ_SR04M_MODE < 3
  /* GPIO + RMT cleanup */
  aj_sr04m_release_rmt_resources(sensor, true);
  aj_sr04m_release_trigger_pin(sensor->trigger_pin);
#else
  /* SW backend owns RMT/GPIO resources; HW backend owns a UART driver. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    aj_sr04m_release_rmt_resources(sensor, true);
    aj_sr04m_release_trigger_pin(sensor->trigger_pin);
  } else {
    uart_driver_delete(sensor->uart_num);
  }
#endif

  sensor->initialized = false;
  memset(sensor, 0, sizeof(*sensor));
}

static aj_sr04m_handle_t aj_sr04m_get_handle_locked(int index) {
  if (index < 0 || index >= aj_sr04m_registered_count())
    return NULL;

  return s_sensor_handles[index];
}

static esp_err_t aj_sr04m_trigger_all_locked(void) {
  if (!s_initialized || s_sensor_count == 0)
    return ESP_ERR_INVALID_STATE;

  const int count = aj_sr04m_registered_count();
  int triggered = 0;
  esp_err_t last_err = ESP_OK;
  for (int i = 0; i < count; i++) {
    /* Co-located modules hear each other's 40 kHz burst. Firing them back
     * to back makes the neighbour's burst race the real echo, and the
     * loser reports its out-of-range sentinel. Spacing the triggers keeps
     * each measurement window clear. */
    if (i > 0 && AJ_SR04M_TRIGGER_STAGGER_MS > 0)
      vTaskDelay(pdMS_TO_TICKS(AJ_SR04M_TRIGGER_STAGGER_MS));

    const esp_err_t err = aj_sr04m_trigger(s_sensor_handles[i]);
    if (err == ESP_OK) {
      triggered++;
    } else {
      last_err = err;
    }
  }

  /* One sensor failing to arm must not hide the others' measurements, so the
   * loop runs to completion and the error only surfaces when nothing was
   * triggered at all. */
  return triggered > 0 ? ESP_OK : last_err;
}

static esp_err_t aj_sr04m_read_all_locked(int16_t *distances,
                                          aj_sr04m_dist_status_t *statuses,
                                          int max_sensors,
                                          int *out_sensor_count) {
  if (distances == NULL || statuses == NULL || out_sensor_count == NULL)
    return ESP_ERR_INVALID_ARG;
  if (!s_initialized || s_sensor_count == 0)
    return ESP_ERR_INVALID_STATE;
  if (max_sensors < s_sensor_count)
    return ESP_ERR_INVALID_SIZE;

  const int count = aj_sr04m_registered_count();
  for (int i = 0; i < count; i++) {
    statuses[i] = aj_sr04m_read_distance(s_sensor_handles[i], &distances[i]);
  }

  *out_sensor_count = count;
  return ESP_OK;
}

/* Public entry points onto the table. Each one is the lock, the matching
 * _locked helper, and the unlock — nothing else, so no error path can grow
 * its way out of the critical section.
 *
 * aj_sr04m_trigger_all() and aj_sr04m_read_all() hold the lock for their
 * whole run, and both block inside it: the first on the inter-trigger delay,
 * the second on each sensor's capture semaphore. A measurement cycle is
 * therefore serialised against sensor management and against another cycle.
 * That is deliberate: the alternative is a table that changes shape halfway
 * through a walk that lasts milliseconds. The contract is spelled out for
 * callers in the thread-safety section of aj_sr04m.h. */
aj_sr04m_handle_t aj_sr04m_new(int trigger_pin, int echo_pin,
                               uint8_t trigger_byte, int uart_num) {
  aj_sr04m_table_lock();
  aj_sr04m_handle_t handle =
      aj_sr04m_new_locked(trigger_pin, echo_pin, trigger_byte, uart_num);
  aj_sr04m_table_unlock();
  return handle;
}

void aj_sr04m_delete(aj_sr04m_handle_t handle) {
  aj_sr04m_table_lock();
  aj_sr04m_delete_locked(handle);
  aj_sr04m_table_unlock();
}

int aj_sr04m_get_sensor_count(void) {
  aj_sr04m_table_lock();
  const int count = s_sensor_count;
  aj_sr04m_table_unlock();
  return count;
}

aj_sr04m_handle_t aj_sr04m_get_handle(int index) {
  aj_sr04m_table_lock();
  aj_sr04m_handle_t handle = aj_sr04m_get_handle_locked(index);
  aj_sr04m_table_unlock();
  return handle;
}

esp_err_t aj_sr04m_trigger_all(void) {
  aj_sr04m_table_lock();
  const esp_err_t err = aj_sr04m_trigger_all_locked();
  aj_sr04m_table_unlock();
  return err;
}

esp_err_t aj_sr04m_read_all(int16_t *distances,
                            aj_sr04m_dist_status_t *statuses, int max_sensors,
                            int *out_sensor_count) {
  aj_sr04m_table_lock();
  const esp_err_t err = aj_sr04m_read_all_locked(distances, statuses,
                                                 max_sensors, out_sensor_count);
  aj_sr04m_table_unlock();
  return err;
}

/* Neither of the two below takes the table lock: they work on a handle the
 * caller supplies and never look at the table. Adding the lock would buy
 * nothing — the handle is already a raw pointer the caller holds, so a
 * concurrent aj_sr04m_delete() of that same sensor is unsafe with or without
 * it — while making every per-sensor read serialise against every other one.
 * See the thread-safety section of aj_sr04m.h for what that leaves callers
 * responsible for. */
esp_err_t aj_sr04m_trigger(aj_sr04m_handle_t handle) {
  if (handle == NULL)
    return ESP_ERR_INVALID_ARG;

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return ESP_ERR_INVALID_STATE;

#if AJ_SR04M_MODE < 3
  /* No point pulsing TRIG with nothing listening: the echo would be missed
   * anyway, and the burst would only disturb neighbouring modules. */
  ESP_RETURN_ON_ERROR(aj_sr04m_arm_rmt_capture(sensor), AJ_SR04M_TAG,
                      "trigger aborted: RMT capture not armed");

  gpio_set_level(sensor->trigger_pin, 1);
  esp_rom_delay_us(
#if AJ_SR04M_MODE == 1
      15
#else
      1100
#endif
  );
  gpio_set_level(sensor->trigger_pin, 0);
#else
  /* Modes 3-5. Arming and prompting are separate steps: the software backend
   * captures nothing until rmt_receive() runs, which mode 3 needs just as
   * much as the others even though it sends no trigger byte. The hardware
   * backend needs no arming — its UART driver buffers on its own. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    ESP_RETURN_ON_ERROR(aj_sr04m_arm_rmt_capture(sensor), AJ_SR04M_TAG,
                        "trigger aborted: RMT capture not armed");
  }

#if AJ_SR04M_MODE >= 4
  /* Mode 3 is autonomous: the module streams unprompted, so no byte goes
   * out. Modes 4-5 ask for one measurement per trigger byte. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    aj_sr04m_sw_uart_write_byte(sensor->trigger_pin, sensor->trigger_byte);
  } else {
    /* Start the cycle from an empty buffer. A reply that arrived after its
     * read had timed out is still sitting there, and would be returned as
     * this cycle's measurement — leaving every later cycle one reading
     * behind, or, in mode 5, splitting a "Gap=" payload across two reads.
     *
     * Only the prompted modes flush. Mode 3 streams unprompted, so draining
     * its buffer would discard the very frames the read is after. */
    uart_flush_input(sensor->uart_num);
    uart_write_bytes(sensor->uart_num, &sensor->trigger_byte, 1);
  }
#endif
#endif

  return ESP_OK;
}

aj_sr04m_dist_status_t aj_sr04m_read_distance(aj_sr04m_handle_t handle,
                                              int16_t *distance) {
  if (handle == NULL || distance == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return AJ_SR04M_DIST_BAD_FRAME;

#if AJ_SR04M_MODE <= 2
  if (xSemaphoreTake(sensor->rx_done_sem,
                     pdMS_TO_TICKS(AJ_SR04M_RMT_TIMEOUT_MS)) != pdTRUE)
    return AJ_SR04M_DIST_NO_ECHO;

  if (aj_sr04m_capture_truncated(sensor))
    return AJ_SR04M_DIST_BAD_FRAME;

  uint32_t pulse_us = extract_high_pulse_us(sensor);
  if (pulse_us == 0)
    return AJ_SR04M_DIST_NO_ECHO;

  int16_t mm = (int16_t)((float)pulse_us * 0.1715f);
  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = mm;
  return AJ_SR04M_DIST_OK;
#else
  /* Modes 3-5. Software backend: wait for the RMT capture, decode the 9600
   * 8N1 bytes, then reuse the same frame parsers as the hardware backend. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    if (xSemaphoreTake(sensor->rx_done_sem,
                       pdMS_TO_TICKS(AJ_SR04M_SW_UART_CAPTURE_TIMEOUT_MS)) !=
        pdTRUE)
      return AJ_SR04M_DIST_NO_ECHO;

    if (aj_sr04m_capture_truncated(sensor))
      return AJ_SR04M_DIST_BAD_FRAME;

    uint8_t bytes[64];
    size_t n = aj_sr04m_sw_uart_decode(
        sensor->rx_buffer, sensor->rx_num_symbols, bytes, sizeof(bytes));
#if AJ_SR04M_MODE == 5
    bytes[n < sizeof(bytes) ? n : sizeof(bytes) - 1] = '\0';
    return aj_sr04m_parse_ascii_frame((const char *)bytes, distance);
#else
    return aj_sr04m_parse_binary_stream(bytes, (int)n, distance);
#endif
  }

#if AJ_SR04M_MODE == 5
  /* ASCII frame "Gap=XXXX mm\r\n", reply latency ~100-200 ms */
  char data[64];
  int len = uart_read_bytes(sensor->uart_num, (uint8_t *)data, sizeof(data) - 1,
                            pdMS_TO_TICKS(AJ_SR04M_UART_REPLY_TIMEOUT_MS));
  if (len <= 0)
    return AJ_SR04M_DIST_BAD_FRAME;
  data[len] = '\0';
  return aj_sr04m_parse_ascii_frame(data, distance);
#else
  /* Binary frames, back to back in mode 3. Read a whole buffer rather than a
   * frame's worth: the stream is not delimited, so a read lands mid-frame as
   * often as not, and the scan below picks the newest complete one. */
  uint8_t data[64];
  int len =
      uart_read_bytes(sensor->uart_num, data, sizeof(data),
                      pdMS_TO_TICKS(AJ_SR04M_UART_BINARY_READ_TIMEOUT_MS));
  return aj_sr04m_parse_binary_stream(data, len, distance);
#endif
#endif
}
