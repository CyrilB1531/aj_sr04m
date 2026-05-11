/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Mode 3: autonomous UART — the sensor streams a 4-byte binary frame
 * roughly every 100 ms without prompting. Driver behaviour:
 *   - init configures the UART
 *   - trigger() is a no-op (no trigger byte to send)
 *   - read_duration() reads the next available frame and parses it
 *
 * Variant has no observable effect on mode 3 (no trigger byte is sent).
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_3

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

TEST_CASE("init: mode 3 success path", "[aj_sr04m][init]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_uart_mock.driver_install_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.param_config_calls);
  TEST_ASSERT_EQUAL(1, g_uart_mock.set_pin_calls);
  TEST_ASSERT_EQUAL(9600, g_uart_mock.last_config.baud_rate);
}

TEST_CASE("init: mode 3 fails on uart_driver_install error",
          "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.driver_install_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 3 fails on uart_param_config error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.param_config_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: mode 3 fails on uart_set_pin error", "[aj_sr04m][init]") {
  mocks_reset();
  g_uart_mock.set_pin_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("trigger: mode 3 does not send trigger byte", "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  int before = g_uart_mock.write_bytes_calls;
  aj_sr04m_trigger();
  /* Mode 3 is autonomous: the sensor streams frames on its own, so
   * aj_sr04m_trigger() must be a no-op. */
  TEST_ASSERT_EQUAL(before, g_uart_mock.write_bytes_calls);
}

TEST_CASE("read_duration: mode 3 valid binary frame at 1500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, aj_sr04m_read_duration(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("read_duration: mode 3 BAD_FRAME on wrong header",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0x00, 0x05, 0xDC, 0xE1};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 3 BAD_CHECKSUM on bad checksum",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0x00};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 3 NO_ECHO on out-of-range (6016 mm)",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x17, 0x80, 0x96};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: mode 3 BAD_FRAME on UART read returning 0 bytes",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, aj_sr04m_read_duration(&dist));
}

#endif /* CONFIG_AJ_SR04M_MODE_3 */
