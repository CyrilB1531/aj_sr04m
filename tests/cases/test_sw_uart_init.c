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

#endif /* UART mode on a software-backend port, linux target */
