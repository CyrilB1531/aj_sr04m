/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Mode 4: UART trigger byte + binary frame response.
 *
 * Tests cover init/trigger/read through the mocked UART driver, plus a
 * variant-specific assertion that the trigger byte matches the sensor
 * selected by Kconfig (0x01 for AJ-SR04M, 0x55 for JSN-SR04T).
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_4 && CONFIG_AJ_SR04M_MAX_SENSORS == 1

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

/* === init / trigger / read with mocked UART (mode 4) ===================== */

TEST_CASE("init: mode 4 success path", "[aj_sr04m][init]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_uart_mock.driver_install_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.param_config_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.set_pin_calls);
  TEST_ASSERT_EQUAL(9600, g_uart_mock.last_config.baud_rate);
  TEST_ASSERT_EQUAL(UART_DATA_8_BITS, g_uart_mock.last_config.data_bits);
  TEST_ASSERT_EQUAL(UART_PARITY_DISABLE, g_uart_mock.last_config.parity);
  TEST_ASSERT_EQUAL(UART_STOP_BITS_1, g_uart_mock.last_config.stop_bits);
}

TEST_CASE("init: mode 4 fails on uart_driver_install error",
          "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.driver_install_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 4 fails on uart_param_config error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.param_config_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 4 fails on uart_set_pin error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.set_pin_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

/* The GPIO mock these two cases read is compiled on the linux target only,
 * and the hardware backend is what they exercise — a port number at or above
 * the controller count would route aj_sr04m_new() through the bit-banged one,
 * whose failure exits test_sw_uart_init.c covers. */
#if CONFIG_IDF_TARGET_LINUX && CONFIG_AJ_SR04M_UART_NUM < SOC_UART_NUM

/* Regression for #46. uart_set_pin() routes TX, RX, RTS and CTS one after
 * another and TX goes first, so a failure can return with the trigger pin
 * already wired to the peripheral. Deleting the driver does not undo that
 * routing, and the caller only gets a NULL back: the pin would keep driving
 * a line for a sensor that does not exist, with no handle to release it
 * through. The exit has to park it as a plain input.
 *
 * Asserting the order too, not just that both steps happen: deleting the
 * driver reconfigures the same pins, so a park done first would be written
 * over by the teardown that follows it. */
TEST_CASE("init: mode 4 releases the TX pin when uart_set_pin fails",
          "[aj_sr04m][init]") {
  /* Deinit first, then reset: the teardown log has to start empty, and
   * releasing a sensor left over by an earlier case writes to it. */
  aj_sr04m_deinit();
  mocks_reset();
  g_uart_mock.set_pin_ret = ESP_FAIL;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());
  TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, g_gpio_mock.last_mode);

  const int driver_gone =
      mocks_teardown_step_index(MOCKS_TEARDOWN_UART_DRIVER_DELETE);
  const int pin_parked =
      mocks_teardown_step_index(MOCKS_TEARDOWN_GPIO_PIN_RELEASE);
  TEST_ASSERT_GREATER_OR_EQUAL(0, driver_gone);
  TEST_ASSERT_GREATER_OR_EQUAL(0, pin_parked);
  TEST_ASSERT_LESS_THAN(pin_parked, driver_gone);
}

/* The counterpart, and the reason the exit above is the only one that
 * releases anything: uart_param_config() writes baud rate, frame format and
 * clock source to the peripheral and never touches a pin — routing lives
 * entirely in uart_set_pin(), which has not run yet. Giving back the driver
 * is the whole debt. Parking a pin here would reconfigure a pad this call
 * left exactly as it found it. */
TEST_CASE("init: mode 4 touches no pin when uart_param_config fails",
          "[aj_sr04m][init]") {
  aj_sr04m_deinit();
  mocks_reset();
  g_uart_mock.param_config_ret = ESP_FAIL;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_uart_mock.driver_delete_calls);
  TEST_ASSERT_EQUAL(0, g_gpio_mock.config_calls);
}

#endif /* hardware UART backend, linux target */

TEST_CASE("trigger: mode 4 sends configured trigger byte",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  int before = g_uart_mock.write_bytes_calls;
  aj_sr04m_trigger_all();
  TEST_ASSERT_EQUAL(before + 1, g_uart_mock.write_bytes_calls);
  TEST_ASSERT_EQUAL(CONFIG_AJ_SR04M_TRIGGER_BYTE, g_uart_mock.last_write_byte);
  TEST_ASSERT_EQUAL(1, g_uart_mock.last_write_size);
}

#if CONFIG_AJ_SR04M_VARIANT_AJ_SR04M
TEST_CASE("trigger: mode 4 AJ-SR04M variant uses 0x01",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger_all();
  TEST_ASSERT_EQUAL_HEX8(0x01, g_uart_mock.last_write_byte);
}
#elif CONFIG_AJ_SR04M_VARIANT_JSN_SR04T
TEST_CASE("trigger: mode 4 JSN-SR04T variant uses 0x55",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger_all();
  TEST_ASSERT_EQUAL_HEX8(0x55, g_uart_mock.last_write_byte);
}
#endif

TEST_CASE("read: mode 4 valid binary frame at 1500 mm", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, mocks_read_one(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
  TEST_ASSERT_EQUAL(1, g_uart_mock.read_bytes_calls);
}

TEST_CASE("read: mode 4 BAD_FRAME on wrong header", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0x00, 0x05, 0xDC, 0xE1};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, mocks_read_one(&dist));
}

TEST_CASE("read: mode 4 BAD_CHECKSUM on bad checksum", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0x00};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM, mocks_read_one(&dist));
}

TEST_CASE("read: mode 4 NO_ECHO on out-of-range distance (6016 mm)",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x17, 0x80, 0x96};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

TEST_CASE("read: mode 4 BAD_FRAME on UART read returning 0 bytes",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, mocks_read_one(&dist));
}

/* Regression for #17. A reply that arrives after its read has timed out
 * stays in the driver's ring buffer; without a flush, the next cycle returns
 * it and every later cycle stays one measurement behind. Asserting the
 * ordering, not just the call: a flush after the trigger byte would drop the
 * very reply the cycle is waiting for. */
TEST_CASE("trigger: flushes the input buffer before the trigger byte",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());

  TEST_ASSERT_EQUAL(1, g_uart_mock.write_bytes_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.flush_input_calls);
  /* The flush was already counted when the byte went out. */
  TEST_ASSERT_EQUAL(1, g_uart_mock.flush_calls_at_write);
}

#endif /* CONFIG_AJ_SR04M_MODE_4 */
