/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "mocks.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "aj_sr04m_sw_uart.h"

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
#include "esp_attr.h"

/* esp_rom_delay_us exists on both ESP targets and the linux port
 * (components/esp_rom/linux/esp_rom_sys.c provides it), so we passthrough
 * unconditionally. On ESP targets IDF's cpu_start.c calls it from Core 1
 * before flash cache is enabled, so __wrap_esp_rom_delay_us must live in
 * IRAM (IRAM_ATTR is a no-op on linux). */
extern void __real_esp_rom_delay_us(uint32_t us);

/* esp_driver_gpio is empty on the linux target (no sources), so there is
 * no __real_gpio_config / __real_gpio_set_level to call. The wraps below
 * therefore stub-return on linux and passthrough on ESP targets, where
 * IDF code paths beyond aj_sr04m may also drive GPIO. */
#if !CONFIG_IDF_TARGET_LINUX
extern esp_err_t __real_gpio_config(const gpio_config_t *cfg);
extern esp_err_t __real_gpio_set_level(gpio_num_t pin, uint32_t level);
#endif
#endif

struct uart_mock_state g_uart_mock;

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
struct gpio_mock_state g_gpio_mock;
struct esp_rom_mock_state g_esp_rom_mock;
struct rmt_mock_state g_rmt_mock;

/* One stub handle per channel the driver creates. They are never
 * dereferenced — every RMT call is intercepted here — but they must differ
 * from one another: the driver registers a callback context per sensor, and
 * a completion has to reach the sensor that armed that capture. A single
 * shared handle woke whichever sensor registered last, leaving the others
 * blocked on a capture someone else had consumed. */
#define MOCKS_RMT_MAX_CHANNELS 16
static int s_stub_rmt_channels[MOCKS_RMT_MAX_CHANNELS];
static void *s_stub_rmt_user_data[MOCKS_RMT_MAX_CHANNELS];
static int s_stub_rmt_channels_used;
#endif

void mocks_reset(void) {
  memset(&g_uart_mock, 0, sizeof(g_uart_mock));
  g_uart_mock.driver_install_ret = ESP_OK;
  g_uart_mock.param_config_ret = ESP_OK;
  g_uart_mock.set_pin_ret = ESP_OK;
  g_uart_mock.write_bytes_ret = 1;
  g_uart_mock.read_bytes_ret = 0;
  g_uart_mock.read_buffer = NULL;
  g_uart_mock.read_buffer_len = 0;

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
  memset(&g_gpio_mock, 0, sizeof(g_gpio_mock));
  g_gpio_mock.config_ret = ESP_OK;
  g_gpio_mock.set_level_ret = ESP_OK;

  memset(&g_esp_rom_mock, 0, sizeof(g_esp_rom_mock));

  memset(&g_rmt_mock, 0, sizeof(g_rmt_mock));
  memset(s_stub_rmt_user_data, 0, sizeof(s_stub_rmt_user_data));
  s_stub_rmt_channels_used = 0;
  g_rmt_mock.new_rx_channel_ret = ESP_OK;
  g_rmt_mock.register_event_callbacks_ret = ESP_OK;
  g_rmt_mock.enable_ret = ESP_OK;
  g_rmt_mock.receive_ret = ESP_OK;
#endif

#if CONFIG_IDF_TARGET_LINUX
  memset(&g_heap_mock, 0, sizeof(g_heap_mock));
#endif
}

esp_err_t __wrap_uart_driver_delete(uart_port_t port) {
  (void)port;
  g_uart_mock.driver_delete_calls++;
  return ESP_OK;
}

esp_err_t __wrap_uart_driver_install(uart_port_t port, int rx_buffer_size,
                                     int tx_buffer_size, int queue_size,
                                     void *uart_queue, int intr_alloc_flags) {
  (void)rx_buffer_size;
  (void)tx_buffer_size;
  (void)queue_size;
  (void)uart_queue;
  (void)intr_alloc_flags;
  g_uart_mock.driver_install_calls++;
  g_uart_mock.last_port = port;
  return g_uart_mock.driver_install_ret;
}

esp_err_t __wrap_uart_param_config(uart_port_t port,
                                   const uart_config_t *uart_config) {
  (void)port;
  g_uart_mock.param_config_calls++;
  if (uart_config) {
    g_uart_mock.last_config = *uart_config;
  }
  return g_uart_mock.param_config_ret;
}

esp_err_t __wrap_uart_set_pin(uart_port_t port, int tx_io_num, int rx_io_num,
                              int rts_io_num, int cts_io_num) {
  (void)port;
  (void)rts_io_num;
  (void)cts_io_num;
  g_uart_mock.set_pin_calls++;
  g_uart_mock.last_tx_pin = tx_io_num;
  g_uart_mock.last_rx_pin = rx_io_num;
  return g_uart_mock.set_pin_ret;
}

int __wrap_uart_write_bytes(uart_port_t port, const void *src, size_t size) {
  (void)port;
  g_uart_mock.write_bytes_calls++;
  g_uart_mock.flush_calls_at_write = g_uart_mock.flush_input_calls;
  g_uart_mock.last_write_size = size;
  if (size > 0 && src) {
    g_uart_mock.last_write_byte = ((const uint8_t *)src)[0];
  }
  return g_uart_mock.write_bytes_ret;
}

esp_err_t __wrap_uart_flush_input(uart_port_t port) {
  (void)port;
  g_uart_mock.flush_input_calls++;
  return ESP_OK;
}

int __wrap_uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
                           TickType_t ticks_to_wait) {
  (void)port;
  (void)ticks_to_wait;
  g_uart_mock.read_bytes_calls++;
  if (g_uart_mock.read_buffer && g_uart_mock.read_buffer_len > 0 && buf) {
    int n = (int)length < g_uart_mock.read_buffer_len
                ? (int)length
                : g_uart_mock.read_buffer_len;
    memcpy(buf, g_uart_mock.read_buffer, (size_t)n);
    return n;
  }
  return g_uart_mock.read_bytes_ret;
}

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2

esp_err_t __wrap_gpio_config(const gpio_config_t *cfg) {
  g_gpio_mock.config_calls++;
  if (cfg) {
    g_gpio_mock.last_pin_bit_mask = cfg->pin_bit_mask;
    g_gpio_mock.last_mode = (int)cfg->mode;
  }
  if (g_gpio_mock.config_ret != ESP_OK) {
    return g_gpio_mock.config_ret;
  }
#if CONFIG_IDF_TARGET_LINUX
  return ESP_OK;
#else
  return __real_gpio_config(cfg);
#endif
}

esp_err_t __wrap_gpio_set_level(gpio_num_t pin, uint32_t level) {
  g_gpio_mock.set_level_calls++;
  g_gpio_mock.last_pin = (int)pin;
  g_gpio_mock.last_level = (int)level;
  if (g_gpio_mock.level_seq_len < GPIO_MOCK_MAX_LEVELS) {
    g_gpio_mock.level_seq[g_gpio_mock.level_seq_len++] = (int)level;
  }
  if (g_gpio_mock.set_level_ret != ESP_OK) {
    return g_gpio_mock.set_level_ret;
  }
#if CONFIG_IDF_TARGET_LINUX
  return ESP_OK;
#else
  return __real_gpio_set_level(pin, level);
#endif
}

/* IRAM_ATTR is mandatory: Core 1's early bring-up path calls this
 * (renamed by --wrap) before flash cache is enabled. Without IRAM
 * placement the fetch faults with LoadProhibited. The real ROM
 * implementation must run so cache/PMU timing is preserved. */
void IRAM_ATTR __wrap_esp_rom_delay_us(uint32_t us) {
  const int call = g_esp_rom_mock.delay_us_calls++;
  g_esp_rom_mock.last_delay_us = us;
  if (g_esp_rom_mock.delay_seq_len < ESP_ROM_MOCK_MAX_DELAYS) {
    g_esp_rom_mock.delay_seq[g_esp_rom_mock.delay_seq_len++] = us;
  }

  g_esp_rom_mock.now_us += us;
  if (g_esp_rom_mock.overrun_us > 0 && call == g_esp_rom_mock.overrun_at_call) {
    g_esp_rom_mock.now_us += g_esp_rom_mock.overrun_us;
    g_esp_rom_mock.overrun_us = 0;
  }

  __real_esp_rom_delay_us(us);
}

static int mocks_rmt_channel_index(rmt_channel_handle_t channel) {
  for (int i = 0; i < s_stub_rmt_channels_used; i++) {
    if (channel == (rmt_channel_handle_t)&s_stub_rmt_channels[i])
      return i;
  }
  return -1;
}

esp_err_t __wrap_rmt_new_rx_channel(const rmt_rx_channel_config_t *cfg,
                                    rmt_channel_handle_t *out) {
  (void)cfg;
  g_rmt_mock.new_rx_channel_calls++;
  if (g_rmt_mock.new_rx_channel_ret != ESP_OK)
    return g_rmt_mock.new_rx_channel_ret;

  /* Running out of stubs would silently hand back a channel already in use,
   * so it fails the way the peripheral does when its channels are taken. */
  if (s_stub_rmt_channels_used >= MOCKS_RMT_MAX_CHANNELS)
    return ESP_ERR_NOT_FOUND;

  if (out) {
    *out = (rmt_channel_handle_t)&s_stub_rmt_channels[s_stub_rmt_channels_used];
  }
  s_stub_rmt_channels_used++;
  return ESP_OK;
}

esp_err_t
__wrap_rmt_rx_register_event_callbacks(rmt_channel_handle_t channel,
                                       const rmt_rx_event_callbacks_t *cbs,
                                       void *user_data) {
  g_rmt_mock.register_event_callbacks_calls++;
  if (cbs) {
    g_rmt_mock.on_recv_done = (aj_sr04m_rmt_rx_done_cb_t)cbs->on_recv_done;
  }

  const int index = mocks_rmt_channel_index(channel);
  if (index >= 0) {
    s_stub_rmt_user_data[index] = user_data;
  }
  return g_rmt_mock.register_event_callbacks_ret;
}

esp_err_t __wrap_rmt_enable(rmt_channel_handle_t channel) {
  (void)channel;
  g_rmt_mock.enable_calls++;
  return g_rmt_mock.enable_ret;
}

esp_err_t __wrap_rmt_disable(rmt_channel_handle_t channel) {
  (void)channel;
  return ESP_OK;
}

esp_err_t __wrap_rmt_del_channel(rmt_channel_handle_t channel) {
  (void)channel;
  return ESP_OK;
}

/* Encodes g_rmt_mock.fire_uart_bytes into @p buffer the way the RMT hardware
 * would record the module driving the line: one run per stretch of identical
 * level, two runs per symbol. Returns the number of symbols written, or 0 if
 * the frame does not fit — a silent short write would look like a decode bug
 * rather than a test that asked for too much. */
static size_t mocks_encode_uart_capture(rmt_symbol_word_t *symbols,
                                        size_t capacity) {
  size_t runs = 0;
  int cur_level = -1;

  for (size_t i = 0; i < g_rmt_mock.fire_uart_len; i++) {
    uint8_t levels[AJ_SR04M_SW_UART_FRAME_BITS];
    aj_sr04m_sw_uart_encode_byte(g_rmt_mock.fire_uart_bytes[i], levels);

    for (int b = 0; b < AJ_SR04M_SW_UART_FRAME_BITS; b++) {
      if (levels[b] == cur_level) {
        /* Same level as the previous bit: extend the run in progress rather
         * than open a new one, exactly as the hardware records it. */
        const size_t last = runs - 1;
        if (last % 2 == 0) {
          symbols[last / 2].duration0 += AJ_SR04M_SW_UART_BIT_US;
        } else {
          symbols[last / 2].duration1 += AJ_SR04M_SW_UART_BIT_US;
        }
        continue;
      }

      if (runs / 2 >= capacity)
        return 0;

      rmt_symbol_word_t *sym = &symbols[runs / 2];
      if (runs % 2 == 0) {
        sym->level0 = levels[b];
        sym->duration0 = AJ_SR04M_SW_UART_BIT_US;
      } else {
        sym->level1 = levels[b];
        sym->duration1 = AJ_SR04M_SW_UART_BIT_US;
      }
      cur_level = levels[b];
      runs++;
    }
  }

  return (runs + 1) / 2;
}

esp_err_t __wrap_rmt_receive(rmt_channel_handle_t channel, void *buffer,
                             size_t buffer_size,
                             const rmt_receive_config_t *cfg) {
  (void)channel;
  (void)cfg;
  g_rmt_mock.receive_calls++;
  g_rmt_mock.last_receive_buffer = buffer;
  g_rmt_mock.last_receive_buffer_size = buffer_size;

  /* Synthesise an immediate RX-done event with one high-pulse symbol.
   * This stands in for the hardware that would otherwise call back
   * asynchronously when the echo arrives. The callback gives the
   * semaphore that aj_sr04m_read_duration() blocks on, so it returns
   * without waiting for the timeout. */
  if (g_rmt_mock.fire_pulse_on_receive && g_rmt_mock.on_recv_done && buffer &&
      buffer_size >= sizeof(rmt_symbol_word_t)) {
    const size_t capacity = buffer_size / sizeof(rmt_symbol_word_t);
    rmt_symbol_word_t *sym = (rmt_symbol_word_t *)buffer;
    size_t written;

    memset(sym, 0, buffer_size);
    if (g_rmt_mock.fire_uart_bytes != NULL && g_rmt_mock.fire_uart_len > 0) {
      written = mocks_encode_uart_capture(sym, capacity);
    } else {
      sym->level0 = 1;
      sym->duration0 = (uint16_t)g_rmt_mock.fire_pulse_high_us;
      written = 1;
    }

    rmt_rx_done_event_data_t evt = {
        .num_symbols =
            g_rmt_mock.fire_capture_fills_buffer ? capacity : written,
        .received_symbols = sym,
    };
    const int index = mocks_rmt_channel_index(channel);
    g_rmt_mock.on_recv_done(channel, &evt,
                            index >= 0 ? s_stub_rmt_user_data[index] : NULL);
  }
  return g_rmt_mock.receive_ret;
}

#endif /* hardware-driver mocks (linux all modes, ESP modes 1-2) */

#if CONFIG_IDF_TARGET_LINUX

#include "esp_timer.h"

/* ESP-IDF registers esp_timer on the linux target with headers only — no
 * implementation — so the software UART's edge scheduling would not link
 * here. Backing it with the virtual clock the delay mock advances is also
 * what makes that scheduling testable: the code busy-waits, the clock moves
 * by exactly what it asked for, and an injected overrun is visible in the
 * next delay it requests. */
int64_t esp_timer_get_time(void) { return g_esp_rom_mock.now_us; }

struct heap_mock_state g_heap_mock;

/* Size of the buffer aj_sr04m_new() allocates for an RMT capture. Matching
 * on it keeps __wrap_malloc from failing anyone else's allocation while the
 * flag is armed. Kept in sync with AJ_SR04M_RMT_NUM_SYMBOLS in aj_sr04m.c —
 * a drift makes the injection stop firing, which shows up as the failure
 * cases below no longer failing. */
#define MOCKS_RMT_BUFFER_BYTES (64 * sizeof(rmt_symbol_word_t))

extern void *__real_malloc(size_t size);

void *__wrap_malloc(size_t size) {
  if (size == MOCKS_RMT_BUFFER_BYTES) {
    g_heap_mock.rmt_buffer_alloc_calls++;
    if (g_heap_mock.fail_rmt_buffer_alloc) {
      g_heap_mock.fail_rmt_buffer_alloc = false;
      return NULL;
    }
  }
  return __real_malloc(size);
}

/* xSemaphoreCreateBinary() is a macro over xQueueGenericCreate(), so the
 * wrap goes on the queue entry point and filters by queue type. */
extern QueueHandle_t __real_xQueueGenericCreate(UBaseType_t uxQueueLength,
                                                UBaseType_t uxItemSize,
                                                uint8_t ucQueueType);

QueueHandle_t __wrap_xQueueGenericCreate(UBaseType_t uxQueueLength,
                                         UBaseType_t uxItemSize,
                                         uint8_t ucQueueType) {
  if (ucQueueType == queueQUEUE_TYPE_BINARY_SEMAPHORE) {
    g_heap_mock.semaphore_create_calls++;
    if (g_heap_mock.fail_semaphore_create) {
      g_heap_mock.fail_semaphore_create = false;
      return NULL;
    }
  }
  return __real_xQueueGenericCreate(uxQueueLength, uxItemSize, ucQueueType);
}

#endif /* CONFIG_IDF_TARGET_LINUX */

aj_sr04m_dist_status_t mocks_read_one(int16_t *dist) {
  aj_sr04m_dist_status_t status = AJ_SR04M_DIST_BAD_FRAME;
  int count = 0;
  if (aj_sr04m_read_all(dist, &status, 1, &count) != ESP_OK) {
    return AJ_SR04M_DIST_BAD_FRAME;
  }
  (void)count;
  return status;
}
