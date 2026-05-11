/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Mode 5: UART trigger byte + ASCII "Gap=XXXX mm\r\n" response.
 *
 * Same call sequence as mode 4 (UART init, trigger byte write, then
 * read), but the response is parsed by `aj_sr04m_parse_ascii_frame`. The
 * variant assertion is the same shape — Kconfig wires 0x01 / 0x55 based
 * on AJ-SR04M vs JSN-SR04T.
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_5

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

TEST_CASE("init: mode 5 success path", "[aj_sr04m][init]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_uart_mock.driver_install_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.param_config_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.set_pin_calls);
  TEST_ASSERT_EQUAL(9600, g_uart_mock.last_config.baud_rate);
}

TEST_CASE("init: mode 5 fails on uart_driver_install error",
          "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.driver_install_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 5 fails on uart_param_config error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.param_config_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 5 fails on uart_set_pin error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.set_pin_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("trigger: mode 5 sends configured trigger byte",
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
TEST_CASE("trigger: mode 5 AJ-SR04M variant uses 0x01",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL_HEX8(0x01, g_uart_mock.last_write_byte);
}
#elif CONFIG_AJ_SR04M_VARIANT_JSN_SR04T
TEST_CASE("trigger: mode 5 JSN-SR04T variant uses 0x55",
          "[aj_sr04m][trigger][variant]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL_HEX8(0x55, g_uart_mock.last_write_byte);
}
#endif

TEST_CASE("read_duration: mode 5 valid ASCII frame at 1500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const char frame[] = "Gap=1500 mm\r\n";
  g_uart_mock.read_buffer = (const uint8_t *)frame;
  g_uart_mock.read_buffer_len = (int)sizeof(frame) - 1;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, aj_sr04m_read_duration(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("read_duration: mode 5 BAD_FRAME on missing Gap= pattern",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const char frame[] = "Hello world\r\n";
  g_uart_mock.read_buffer = (const uint8_t *)frame;
  g_uart_mock.read_buffer_len = (int)sizeof(frame) - 1;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 5 NO_ECHO on out-of-range (6016 mm)",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const char frame[] = "Gap=6016 mm\r\n";
  g_uart_mock.read_buffer = (const uint8_t *)frame;
  g_uart_mock.read_buffer_len = (int)sizeof(frame) - 1;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 5 BAD_FRAME on UART read returning 0 bytes",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

#endif /* CONFIG_AJ_SR04M_MODE_5 */
