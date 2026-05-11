/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Mode 4: UART trigger byte + binary frame response.
 *
 * Tests cover init/trigger/read_duration through the mocked UART driver,
 * plus a variant-specific assertion that the trigger byte matches the
 * sensor selected by Kconfig (0x01 for AJ-SR04M, 0x55 for JSN-SR04T).
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_4

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

/* === init / trigger / read_duration with mocked UART (mode 4) ============ */

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

TEST_CASE("trigger: mode 4 sends configured trigger byte",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  int before = g_uart_mock.write_bytes_calls;
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL(before + 1, g_uart_mock.write_bytes_calls);
  TEST_ASSERT_EQUAL(CONFIG_AJ_SR04M_TRIGGER_BYTE, g_uart_mock.last_write_byte);
  TEST_ASSERT_EQUAL(1, g_uart_mock.last_write_size);
}

#if CONFIG_AJ_SR04M_VARIANT_AJ_SR04M
TEST_CASE("trigger: mode 4 AJ-SR04M variant uses 0x01",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL_HEX8(0x01, g_uart_mock.last_write_byte);
}
#elif CONFIG_AJ_SR04M_VARIANT_JSN_SR04T
TEST_CASE("trigger: mode 4 JSN-SR04T variant uses 0x55",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL_HEX8(0x55, g_uart_mock.last_write_byte);
}
#endif

TEST_CASE("read_duration: mode 4 valid binary frame at 1500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, aj_sr04m_read_duration(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
  TEST_ASSERT_EQUAL(1, g_uart_mock.read_bytes_calls);
}

TEST_CASE("read_duration: mode 4 BAD_FRAME on wrong header",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0x00, 0x05, 0xDC, 0xE1};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 4 BAD_CHECKSUM on bad checksum",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0x00};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 4 NO_ECHO on out-of-range distance (6016 mm)",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x17, 0x80, 0x96};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 4 BAD_FRAME on UART read returning 0 bytes",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

#endif /* CONFIG_AJ_SR04M_MODE_4 */
