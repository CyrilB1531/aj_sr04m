/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Multi-sensor driver management.
 *
 * Compiled only when Kconfig configures more than one sensor, which the
 * `sdkconfig.defaults.multi` fragment does. These cases cover what stays
 * unreachable at a single sensor: the loops over the handle array, the
 * trigger stagger, the removal shift and the pool-exhaustion branch.
 *
 * Assertions stay mode-agnostic -- they exercise the driver's bookkeeping
 * rather than a wire protocol, so the same file runs under every mode
 * fragment. Per-mode wire behaviour is covered by the test_mode*.c files.
 */

#include "sdkconfig.h"

#if CONFIG_AJ_SR04M_MAX_SENSORS > 1

#include <stdint.h>

#include "esp_err.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

#define EXPECTED_SENSORS CONFIG_AJ_SR04M_MAX_SENSORS

/* === driver lifecycle ==================================================== */

TEST_CASE("multi: init creates every configured sensor", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, aj_sr04m_get_sensor_count());
}

TEST_CASE("multi: init is idempotent", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, aj_sr04m_get_sensor_count());
}

TEST_CASE("multi: deinit releases every sensor", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_deinit());
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());
}

TEST_CASE("multi: new returns NULL once the pool is full",
          "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, aj_sr04m_get_sensor_count());

  /* Every slot is taken by the Kconfig-driven instances. */
  TEST_ASSERT_NULL(aj_sr04m_new(25, 26, 0x01, 0));
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, aj_sr04m_get_sensor_count());
}

/* === handle removal ====================================================== */

TEST_CASE("multi: deleting the first sensor shifts the remaining handles",
          "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  /* Removing index 0 forces the shift pass rather than a plain pop. */
  aj_sr04m_delete(aj_sr04m_get_handle(0));
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS - 1, aj_sr04m_get_sensor_count());

  /* The survivor moved down and is still usable. */
  TEST_ASSERT_NOT_NULL(aj_sr04m_get_handle(0));
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
}

TEST_CASE("multi: deleting the last sensor leaves the array consistent",
          "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  aj_sr04m_delete(aj_sr04m_get_handle(EXPECTED_SENSORS - 1));
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS - 1, aj_sr04m_get_sensor_count());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
}

TEST_CASE("multi: deleting an unknown handle changes nothing",
          "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  aj_sr04m_delete(NULL);
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, aj_sr04m_get_sensor_count());
}

/* === bulk trigger / read ================================================= */

TEST_CASE("multi: trigger_all walks every sensor", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
}

TEST_CASE("multi: read_all reports one status per sensor",
          "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  int16_t distances[EXPECTED_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[EXPECTED_SENSORS] = {0};
  int count = 0;

  TEST_ASSERT_EQUAL(
      ESP_OK, aj_sr04m_read_all(distances, statuses, EXPECTED_SENSORS, &count));
  TEST_ASSERT_EQUAL(EXPECTED_SENSORS, count);
}

TEST_CASE("multi: read_all rejects a too small array", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  int16_t distances[EXPECTED_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[EXPECTED_SENSORS] = {0};
  int count = 0;

  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_SIZE,
      aj_sr04m_read_all(distances, statuses, EXPECTED_SENSORS - 1, &count));
}

TEST_CASE("multi: read_all rejects NULL arguments", "[aj_sr04m][multi]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  int16_t distances[EXPECTED_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[EXPECTED_SENSORS] = {0};
  int count = 0;

  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(NULL, statuses, EXPECTED_SENSORS, &count));
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(distances, NULL, EXPECTED_SENSORS, &count));
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(distances, statuses, EXPECTED_SENSORS, NULL));
}

/* === calls made before init ============================================== */

TEST_CASE("multi: bulk calls fail while the driver is down",
          "[aj_sr04m][multi]") {
  mocks_reset();
  aj_sr04m_deinit();

  int16_t distances[EXPECTED_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[EXPECTED_SENSORS] = {0};
  int count = 0;

  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, aj_sr04m_trigger_all());
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_STATE,
      aj_sr04m_read_all(distances, statuses, EXPECTED_SENSORS, &count));
}

#endif /* CONFIG_AJ_SR04M_MAX_SENSORS > 1 */
