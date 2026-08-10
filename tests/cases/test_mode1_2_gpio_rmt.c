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

#if (CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2) &&                      \
    CONFIG_AJ_SR04M_MAX_SENSORS == 1

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

  aj_sr04m_trigger_all();

  /* Triggering arms RMT before raising the pin. */
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
  aj_sr04m_trigger_all();
  TEST_ASSERT_EQUAL(1, g_esp_rom_mock.delay_us_calls);
  TEST_ASSERT_EQUAL(15, g_esp_rom_mock.last_delay_us);
}
#endif

#if AJ_SR04M_MODE == 2
TEST_CASE("trigger: mode 2 pulse width is 1100 us", "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_init();
  aj_sr04m_trigger_all();
  TEST_ASSERT_EQUAL(1, g_esp_rom_mock.delay_us_calls);
  TEST_ASSERT_EQUAL(1100, g_esp_rom_mock.last_delay_us);
}
#endif

TEST_CASE("read: OK with synthetic 8746 us pulse -> ~1500 mm",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* 0.1715 mm/µs × 8746 µs ≈ 1500 mm */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 8746;

  aj_sr04m_init();
  aj_sr04m_trigger_all();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, mocks_read_one(&dist));
  /* Allow ±1 mm slop from float truncation in the driver. */
  TEST_ASSERT_INT16_WITHIN(1, 1500, dist);
}

TEST_CASE("read: NO_ECHO when pulse maps below 200 mm", "[aj_sr04m][read]") {
  mocks_reset();
  /* 100 µs → ~17 mm, well under the 200 mm minimum. */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 100;

  aj_sr04m_init();
  aj_sr04m_trigger_all();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

TEST_CASE("read: NO_ECHO when pulse maps above 4500 mm", "[aj_sr04m][read]") {
  mocks_reset();
  /* 30000 µs → ~5145 mm, beyond the 4500 mm cap. */
  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 30000;

  aj_sr04m_init();
  aj_sr04m_trigger_all();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

TEST_CASE("read: NO_ECHO when no callback fires (semaphore timeout)",
          "[aj_sr04m][read]") {
  mocks_reset();
  /* fire_pulse_on_receive stays false → rmt_receive does not invoke the
   * callback → xSemaphoreTake times out (50 ms) → NO_ECHO. */
  aj_sr04m_init();
  aj_sr04m_trigger_all();
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

/* Regression for #19. A capture that completes after its read gave up used
 * to leave rx_done_sem signalled, and nothing cleared it before the next
 * arming: the following read took that stale give straight away and decoded
 * rx_buffer while RMT was writing the new capture into it. Here cycle 1
 * completes and is deliberately never read; cycle 2 captures nothing, so a
 * correct driver must block and time out rather than hand back cycle 1's
 * measurement. */
TEST_CASE("trigger: drops a completion left by a timed-out cycle",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  /* Fresh driver state: rx_done_sem must not carry over from another case. */
  aj_sr04m_deinit();
  aj_sr04m_init();

  g_rmt_mock.fire_pulse_on_receive = true;
  g_rmt_mock.fire_pulse_high_us = 8746; /* ~1500 mm */
  aj_sr04m_trigger_all();
  /* The caller never reads — its read had already timed out. */

  g_rmt_mock.fire_pulse_on_receive = false; /* cycle 2 captures nothing */
  aj_sr04m_trigger_all();

  int16_t dist = -1;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO, mocks_read_one(&dist));
}

/* Regression for #20. A failed rmt_receive() used to be discarded: the pulse
 * went out with no receiver listening, so the read that followed timed out and
 * reported NO_ECHO — a driver fault wearing the same status as a sensor
 * pointing at open air. The trigger must now fail loudly and leave TRIG
 * untouched. */
TEST_CASE("trigger: reports the failure when RMT cannot be armed",
          "[aj_sr04m][trigger]") {
  mocks_reset();
  aj_sr04m_deinit();
  aj_sr04m_init();

  const int gpio_calls_before = g_gpio_mock.set_level_calls;
  g_rmt_mock.receive_ret = ESP_FAIL;

  /* The only sensor failed to arm, so the batch has nothing to report. */
  TEST_ASSERT_EQUAL(ESP_FAIL, aj_sr04m_trigger_all());
  /* No burst went out: the pin never moved. */
  TEST_ASSERT_EQUAL(gpio_calls_before, g_gpio_mock.set_level_calls);
  TEST_ASSERT_EQUAL(0, g_esp_rom_mock.delay_us_calls);
}

TEST_CASE("trigger: rejects a NULL handle", "[aj_sr04m][trigger]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, aj_sr04m_trigger(NULL));
}

#if CONFIG_IDF_TARGET_LINUX
/* The two aj_sr04m_new() exits no driver mock can reach. They matter more
 * than their length suggests: each one runs the cleanup that releases what
 * the setup had already acquired, so an untested exit is an untested
 * release. Injection is linux-only — see tests/linux/main/CMakeLists.txt. */
TEST_CASE("init: fails when the RMT buffer allocation fails",
          "[aj_sr04m][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  g_heap_mock.fail_rmt_buffer_alloc = true;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_heap_mock.rmt_buffer_alloc_calls);
  /* Setup got as far as driving the trigger pin, so it must have handed it
   * back before giving up. */
  TEST_ASSERT_EQUAL(GPIO_MODE_INPUT, g_gpio_mock.last_mode);
}

TEST_CASE("init: fails when the RMT semaphore cannot be created",
          "[aj_sr04m][init]") {
  mocks_reset();
  aj_sr04m_deinit();
  g_heap_mock.fail_semaphore_create = true;

  TEST_ASSERT_NOT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(1, g_heap_mock.semaphore_create_calls);
}
#endif /* CONFIG_IDF_TARGET_LINUX */

#endif /* CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2 */
