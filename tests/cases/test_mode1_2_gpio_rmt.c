/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Modes 1 & 2: GPIO trigger + RMT echo capture.
 *
 * The init/read paths are identical between modes 1 and 2; only the
 * trigger pulse width differs (15 µs for mode 1 vs 1100 µs for mode 2),
 * so the per-mode pulse-width assertion is split via #if AJ_SR04M_MODE.
 *
 * The variant flag (AJ-SR04M vs JSN-SR04T) has no observable effect in
 * these modes — there is no trigger byte to send.
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

TEST_CASE("init: GPIO+RMT success path", "[aj_sr04m][init]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_gpio_mock.config_calls);
  TEST_ASSERT_EQUAL(1, g_gpio_mock.set_level_calls);
  TEST_ASSERT_EQUAL(0, g_gpio_mock.last_level); /* trigger driven low */
  TEST_ASSERT_EQUAL(1, g_rmt_mock.new_rx_channel_calls);
  TEST_ASSERT_EQUAL(1, g_rmt_mock.register_event_callbacks_calls);
  TEST_ASSERT_EQUAL(1, g_rmt_mock.enable_calls);
}

TEST_CASE("init: fails on gpio_config error", "[aj_sr04m][init]") {
  mocks_reset();
  g_gpio_mock.config_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: fails on gpio_set_level error", "[aj_sr04m][init]") {
  mocks_reset();
  g_gpio_mock.set_level_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: fails on rmt_new_rx_channel error", "[aj_sr04m][init]") {
  mocks_reset();
  g_rmt_mock.new_rx_channel_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: fails on rmt_rx_register_event_callbacks error",
          "[aj_sr04m][init]") {
  mocks_reset();
  g_rmt_mock.register_event_callbacks_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("init: fails on rmt_enable error", "[aj_sr04m][init]") {
  mocks_reset();
  g_rmt_mock.enable_ret = ESP_FAIL;
  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
}

TEST_CASE("trigger: drives GPIO high then low and starts RMT receive",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  int rx_calls_before = g_rmt_mock.receive_calls;
  int gpio_calls_before = g_gpio_mock.set_level_calls;

  aj_sr04m_trigger();

  /* aj_sr04m_trigger arms RMT before raising the pin. */
  TEST_ASSERT_EQUAL(rx_calls_before + 1, g_rmt_mock.receive_calls);
  /* Two gpio_set_level calls: trigger high then trigger low. */
  TEST_ASSERT_EQUAL(gpio_calls_before + 2, g_gpio_mock.set_level_calls);
  /* Final level recorded is the trailing edge (low). */
  TEST_ASSERT_EQUAL(0, g_gpio_mock.last_level);
}

#if AJ_SR04M_MODE == 1
TEST_CASE("trigger: mode 1 pulse width is 15 us", "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL(1, g_esp_rom_mock.delay_us_calls);
  TEST_ASSERT_EQUAL(15, g_esp_rom_mock.last_delay_us);
}
#endif

#if AJ_SR04M_MODE == 2
TEST_CASE("trigger: mode 2 pulse width is 1100 us", "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger();
  TEST_ASSERT_EQUAL(1, g_esp_rom_mock.delay_us_calls);
  TEST_ASSERT_EQUAL(1100, g_esp_rom_mock.last_delay_us);
}
#endif

TEST_CASE("read_duration: OK with synthetic 8746 us pulse -> ~1500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* 0.1715 mm/µs × 8746 µs ≈ 1500 mm */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 8746;

  aj_sr04m_init();
  aj_sr04m_trigger();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, aj_sr04m_read_duration(&dist));
  /* Allow ±1 mm slop from float truncation in the driver. */
  TEST_ASSERT_INT16_WITHIN(1, 1500, dist);
}

TEST_CASE("read_duration: NO_ECHO when pulse maps below 200 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* 100 µs → ~17 mm, well under the 200 mm minimum. */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 100;

  aj_sr04m_init();
  aj_sr04m_trigger();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: NO_ECHO when pulse maps above 4500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* 30000 µs → ~5145 mm, beyond the 4500 mm cap. */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 30000;

  aj_sr04m_init();
  aj_sr04m_trigger();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

TEST_CASE("read_duration: NO_ECHO when no callback fires (semaphore timeout)",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* fire_pulse_on_receive stays false → rmt_receive does not invoke the
   * callback → xSemaphoreTake times out (50 ms) → NO_ECHO. */
  aj_sr04m_init();
  aj_sr04m_trigger();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, aj_sr04m_read_duration(&dist));
}

#endif /* CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2 */
