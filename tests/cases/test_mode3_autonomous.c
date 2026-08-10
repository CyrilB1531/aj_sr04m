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
 *   - read returns the next available frame and parses it
 *
 * Variant has no observable effect on mode 3 (no trigger byte is sent).
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_3 && CONFIG_AJ_SR04M_MAX_SENSORS == 1

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
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
  aj_sr04m_trigger_all();
  /* Mode 3 is autonomous: the sensor streams frames on its own, so triggering
   * must be a no-op. */
  TEST_ASSERT_EQUAL(before, g_uart_mock.write_bytes_calls);
}

TEST_CASE("read: mode 3 valid binary frame at 1500 mm", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, mocks_read_one(&dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("read: mode 3 BAD_FRAME on wrong header", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0x00, 0x05, 0xDC, 0xE1};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, mocks_read_one(&dist));
}

TEST_CASE("read: mode 3 BAD_CHECKSUM on bad checksum", "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0x00};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM, mocks_read_one(&dist));
}

TEST_CASE("read: mode 3 NO_ECHO on out-of-range (6016 mm)",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t frame[4] = {0xFF, 0x17, 0x80, 0x96};
  g_uart_mock.read_buffer = frame;
  g_uart_mock.read_buffer_len = 4;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

TEST_CASE("read: mode 3 BAD_FRAME on UART read returning 0 bytes",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME, mocks_read_one(&dist));
  /* A window that expired with nothing in it is still a bad frame: the read
   * cannot tell an empty buffer from a corrupt one, so the fix below widens
   * the window rather than changing what an empty one reports. */
  TEST_ASSERT_EQUAL(1, g_uart_mock.read_bytes_calls);
}

/* Regression for #45. Mode 3 is autonomous: the module streams a frame about
 * every 120 ms and nothing the driver does brings the next one forward. The
 * read used to open a 20 ms window, which lands between two frames far more
 * often than it lands on one — and an empty read is reported as BAD_FRAME,
 * i.e. a malformed-frame status for a sensor working exactly as specified.
 * The window must be able to outlast a whole stream period. */
TEST_CASE("read: mode 3 waits at least one stream period for a frame",
          "[aj_sr04m][read]") {
  mocks_reset();
  g_uart_mock.read_bytes_ret = 0;
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  int16_t dist = 0;
  mocks_read_one(&dist);

  TEST_ASSERT_EQUAL(1, g_uart_mock.read_bytes_calls);
  TEST_ASSERT_GREATER_OR_EQUAL(pdMS_TO_TICKS(120), g_uart_mock.last_read_ticks);
  /* Upper bound: aj_sr04m_read_all() reads sensors one after another, so a
   * window nobody bounded multiplies by the sensor count on a silent bus. */
  TEST_ASSERT_LESS_OR_EQUAL(pdMS_TO_TICKS(300), g_uart_mock.last_read_ticks);
}

/* Regression for #15. An autonomous module streams a frame every ~100 ms
 * with nothing delimiting the reads, so the driver's buffer holds several
 * frames and starts and ends mid-frame. Reads used to demand a buffer whose
 * length was exactly one frame, which that stream never produces — so every
 * mode 3 read returned BAD_FRAME on real hardware while this file's
 * 4-byte-buffer cases passed. */
TEST_CASE("read: mode 3 finds the newest frame in a streamed buffer",
          "[aj_sr04m][read]") {
  mocks_reset();
  static const uint8_t stream[15] = {
      0xDC, 0xE0,                   /* tail of a frame that started earlier */
      0xFF, 0x05, 0xDC, 0xE0,       /* 1500 mm */
      0xFF, 0x07, 0xD0, 0xD6,       /* 2000 mm */
      0xFF, 0x03, 0xE8, 0xEA, 0xFF, /* 1000 mm (newest), then a bare header */
  };
  g_uart_mock.read_buffer = stream;
  g_uart_mock.read_buffer_len = sizeof(stream);
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, mocks_read_one(&dist));
  TEST_ASSERT_EQUAL_INT16(1000, dist);
}

#endif /* CONFIG_AJ_SR04M_MODE_3 */
