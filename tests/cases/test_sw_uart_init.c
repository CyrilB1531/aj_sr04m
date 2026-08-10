/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Software UART backend setup failures.
 *
 * Compiled only when a UART mode (3-5) is paired with a port number at or
 * above the chip's UART controller count, which the
 * `sdkconfig.defaults.swuart` fragment does: that combination is the only
 * one routing aj_sr04m_new() through the bit-banged TX + RMT RX backend.
 * The hardware-backend failures are covered by the test_mode*.c files, and
 * the GPIO+RMT ones by test_mode1_2_gpio_rmt.c.
 *
 * Linux-only: the GPIO and RMT mocks these cases inject through are
 * compiled on ESP targets in modes 1-2 only.
 */

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_LINUX &&                                                 \
    (CONFIG_AJ_SR04M_MODE_3 || CONFIG_AJ_SR04M_MODE_4 ||                       \
     CONFIG_AJ_SR04M_MODE_5) &&                                                \
    CONFIG_AJ_SR04M_UART_NUM >= SOC_UART_NUM

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

#include "driver/gpio.h"

#include "aj_sr04m.h"
#include "mocks.h"

/* Every case below asserts the same two things: setup reports the failure
 * instead of handing back a half-built sensor, and the TX pin is left as a
 * high-impedance input rather than driven by a sensor that does not exist.
 * The pin check reads the last gpio_config() the driver made, which on a
 * failed setup is the release performed on the way out. */
static void assert_init_fails_and_releases_tx_pin(void) {
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, g_gpio_mock.last_mode);
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());
}

TEST_CASE("sw uart init: fails on rmt_new_rx_channel error",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  g_rmt_mock.new_rx_channel_ret = ESP_FAIL;
  assert_init_fails_and_releases_tx_pin();
}

TEST_CASE("sw uart init: fails on rmt_rx_register_event_callbacks error",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  g_rmt_mock.register_event_callbacks_ret = ESP_FAIL;
  assert_init_fails_and_releases_tx_pin();
}

TEST_CASE("sw uart init: fails on rmt_enable error",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  g_rmt_mock.enable_ret = ESP_FAIL;
  assert_init_fails_and_releases_tx_pin();
}

/* Regression for #38, software-backend side — the one that made the window
 * reachable in ordinary use, mode 3 especially: the module streams
 * unprompted, so a capture is in flight essentially all the time and a
 * shutdown lands in the middle of one. Deleting rx_done_sem while the
 * channel was still enabled left rmt_rx_done_cb() giving a freed handle from
 * an ISR. The channel must be disabled and deleted first. */
TEST_CASE("sw uart deinit: disables the RMT channel before freeing what the "
          "ISR touches",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  /* Only the release matters here, not the semaphore setup created. */
  g_teardown_mock.steps_len = 0;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_deinit());

  const int disabled = mocks_teardown_step_index(MOCKS_TEARDOWN_RMT_DISABLE);
  const int deleted = mocks_teardown_step_index(MOCKS_TEARDOWN_RMT_DEL_CHANNEL);
  const int sem_freed = mocks_teardown_step_index(MOCKS_TEARDOWN_SEM_DELETE);

  TEST_ASSERT_GREATER_OR_EQUAL(0, disabled);
  TEST_ASSERT_GREATER_OR_EQUAL(0, deleted);
  TEST_ASSERT_GREATER_OR_EQUAL(0, sem_freed);
  TEST_ASSERT_LESS_THAN(sem_freed, disabled);
  TEST_ASSERT_LESS_THAN(sem_freed, deleted);
}

TEST_CASE("sw uart deinit: releases the TX pin", "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(GPIO_MODE_OUTPUT, g_gpio_mock.last_mode);

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_deinit());
  TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, g_gpio_mock.last_mode);
}

/* Regression for #16. Arming used to be compiled in only for modes 4-5,
 * while the matching read path covers every UART mode: a mode 3 sensor on a
 * software port waited on a receiver nothing had started, so every read
 * timed out as NO_ECHO. Reachable from the default Kconfig, where sensors 3
 * and 4 sit on ports above SOC_UART_NUM. */
TEST_CASE("sw uart trigger: arms the RMT receiver in every UART mode",
          "[aj_sr04m][sw_uart][trigger]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  const int received_before = g_rmt_mock.receive_calls;
  const int written_before = g_uart_mock.write_bytes_calls;

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());

  TEST_ASSERT_GREATER_THAN(received_before, g_rmt_mock.receive_calls);
#if CONFIG_AJ_SR04M_MODE_3
  /* Autonomous: arming happens, prompting must not. */
  TEST_ASSERT_EQUAL(written_before, g_uart_mock.write_bytes_calls);
#else
  (void)written_before;
#endif
}

/* Regression for #37, software-backend side. The idle threshold is what
 * delimits a frame here, so it has to outlast any gap the module leaves
 * inside one reply — the whole 13-byte mode 5 payload takes 13.5 ms at 9600
 * baud — while staying inside the 15-bit RMT duration counter, 32767 ticks at
 * the 1 MHz resolution. This capture kind is sized independently of the modes
 * 1-2 echo one, which answers to a different worst case. */
TEST_CASE("sw uart trigger: arms the frame capture with a usable idle "
          "threshold",
          "[aj_sr04m][sw_uart][trigger]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());

  TEST_ASSERT_GREATER_THAN_UINT32(13500u * 1000u,
                                  g_rmt_mock.last_signal_range_max_ns);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(32767u * 1000u,
                                   g_rmt_mock.last_signal_range_max_ns);
}

/* Regression for #20, software-backend side. Arming and prompting are two
 * steps here, and a failed arm used to let the second one run anyway: the
 * module answered into a receiver that had never been started, and the read
 * reported NO_ECHO as though the tank were empty. Both sensors fail to arm,
 * so the batch has nothing left to report. */
TEST_CASE("sw uart trigger: reports the failure when RMT cannot be armed",
          "[aj_sr04m][sw_uart][trigger]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  const int written_before = g_uart_mock.write_bytes_calls;
  g_rmt_mock.receive_ret = ESP_FAIL;

  TEST_ASSERT_EQUAL(ESP_FAIL, aj_sr04m_trigger_all());
  /* Nothing was prompted: the module stays quiet instead of answering into
   * a receiver that is not listening. */
  TEST_ASSERT_EQUAL(written_before, g_uart_mock.write_bytes_calls);
}

/* The frame the module would put on the wire for a 1500 mm reading, in the
 * shape the configured mode expects. The mock encodes it as a 9600 8N1
 * capture, so these two cases exercise the whole software path: RMT symbols
 * in, decoded bytes out, parsed distance back. */
#if CONFIG_AJ_SR04M_MODE_5
static const uint8_t k_frame_1500mm[] = "Gap=1500 mm\r\n";
#define K_FRAME_LEN (sizeof(k_frame_1500mm) - 1) /* drop the NUL */
#else
/* header | dist_H | dist_L | checksum, 1500 = 0x05DC */
static const uint8_t k_frame_1500mm[] = {0xFF, 0x05, 0xDC, 0xE0};
#define K_FRAME_LEN sizeof(k_frame_1500mm)
#endif

/* This fragment configures two sensors, so mocks_read_one() — which passes a
 * one-slot array to aj_sr04m_read_all() — would be turned away with
 * ESP_ERR_INVALID_SIZE before any decoding happened. Read the first sensor
 * through its own handle instead. */
static aj_sr04m_dist_status_t read_first_sensor(int16_t *dist) {
  aj_sr04m_handle_t handle = aj_sr04m_get_handle(0);
  TEST_ASSERT_NOT_NULL(handle);
  return aj_sr04m_read_distance(handle, dist);
}

TEST_CASE("sw uart read: decodes a captured frame", "[aj_sr04m][sw_uart]") {
  mocks_reset();
  aj_sr04m_deinit();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_uart_bytes = k_frame_1500mm;
  g_rmt_mock.fire_uart_len = K_FRAME_LEN;

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, read_first_sensor(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

/* Regression for #21. Same frame, same decode — the one thing that changes
 * is the completion reporting a capture that reached the end of the buffer.
 * The RMT engine stops there and hands back what it stored, so the tail is
 * missing and whatever decodes is a fragment. The driver must refuse it
 * rather than hand back the distance the fragment happens to yield. */
TEST_CASE("sw uart read: BAD_FRAME when the capture fills the buffer",
          "[aj_sr04m][sw_uart]") {
  mocks_reset();
  aj_sr04m_deinit();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_uart_bytes = k_frame_1500mm;
  g_rmt_mock.fire_uart_len = K_FRAME_LEN;
  g_rmt_mock.fire_capture_fills_buffer = true;

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
  int16_t dist = -1;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, read_first_sensor(&dist));
}

/* Same two exits on the software backend, which allocates and creates its
 * own capture resources rather than sharing the modes 1-2 ones. */
TEST_CASE("sw uart init: fails when the RX buffer allocation fails",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  g_heap_mock.fail_rmt_buffer_alloc = true;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_heap_mock.rmt_buffer_alloc_calls);
}

TEST_CASE("sw uart init: fails when the RMT semaphore cannot be created",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  g_heap_mock.fail_semaphore_create = true;

  assert_init_fails_and_releases_tx_pin();
  TEST_ASSERT_EQUAL(1, g_heap_mock.semaphore_create_calls);
}

TEST_CASE("sw uart new: returns NULL before the driver is initialized",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  aj_sr04m_deinit();

  TEST_ASSERT_NULL(
      aj_sr04m_new(CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
                   CONFIG_AJ_SR04M_TRIGGER_BYTE, CONFIG_AJ_SR04M_UART_NUM));
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());
}

TEST_CASE("sw uart init: fails on TX pin configuration error",
          "[aj_sr04m][sw_uart][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  g_gpio_mock.config_ret = ESP_FAIL;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());
}

/* Regression for #18. The software backend used to wait
 * AJ_SR04M_RMT_TIMEOUT_MS — 50 ms, sized for a modes 1-2 echo round trip —
 * for a module reply that lands ~100-200 ms after the trigger byte, plus the
 * 30 ms of line idle RMT needs before it reports the capture. Mode 5 over
 * this backend therefore returned NO_ECHO whatever the sensor measured.
 *
 * Asserting on elapsed time rather than on the constant: what matters is
 * that a read actually stays available long enough for a real reply. With
 * nothing ever completing the capture, each sensor burns its full timeout,
 * so the old 50 ms budget cannot reach the bound below. */
TEST_CASE("sw uart read: waits long enough for a module reply",
          "[aj_sr04m][sw_uart][read]") {
  mocks_reset();
  aj_sr04m_deinit();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());

  int16_t distances[CONFIG_AJ_SR04M_MAX_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[CONFIG_AJ_SR04M_MAX_SENSORS] = {0};
  int count = 0;

  const TickType_t started = xTaskGetTickCount();
  TEST_ASSERT_EQUAL(ESP_OK,
                    aj_sr04m_read_all(distances, statuses,
                                      CONFIG_AJ_SR04M_MAX_SENSORS, &count));
  const TickType_t elapsed = xTaskGetTickCount() - started;

  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, statuses[0]);
  /* Well above the old 50 ms per sensor, well below one new timeout, so the
   * bound holds however many sensors the fragment configures. */
  TEST_ASSERT_GREATER_OR_EQUAL(pdMS_TO_TICKS(200), elapsed);
}

#endif /* UART mode on a software-backend port, linux target */
