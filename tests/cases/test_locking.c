/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Sensor-table locking.
 *
 * The driver serialises every access to its sensor table behind one mutex.
 * These cases check the two properties that make the lock worth having:
 * every exit path releases it, and it outlives aj_sr04m_deinit(). They are
 * mode-agnostic -- nothing here touches a wire protocol -- so the file
 * compiles and runs under every fragment of the test matrix.
 */

#include "sdkconfig.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "unity.h"

#include "aj_sr04m.h"
#include "mocks.h"

/* Modes 1-2 ignore the port argument and their Kconfig does not define the
 * symbol; modes 3-5 reuse the configured one so a sensor created here lands
 * on the same backend (hardware or bit-banged) as the Kconfig-driven ones. */
#ifdef CONFIG_AJ_SR04M_UART_NUM
#define LOCK_TEST_UART_PORT CONFIG_AJ_SR04M_UART_NUM
#else
#define LOCK_TEST_UART_PORT 0
#endif

/* Each step below leaves an entry point through one of its early returns,
 * and every step is followed by another call needing the same lock. A return
 * that skipped the unlock does not trip an assertion -- it blocks forever on
 * the next call -- so the symptom of a regression here is a suite that stops
 * on this case rather than one that reports a failure. */
TEST_CASE("locking: every early return releases the table lock",
          "[aj_sr04m][locking]") {
  mocks_reset();
  aj_sr04m_deinit();

  int16_t distances[AJ_SR04M_MAX_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[AJ_SR04M_MAX_SENSORS] = {0};
  int count = -1;

  /* Bulk calls refused because the driver is down. */
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, aj_sr04m_trigger_all());
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_STATE,
      aj_sr04m_read_all(distances, statuses, AJ_SR04M_MAX_SENSORS, &count));

  /* new() before init(), delete(NULL), and an out-of-range lookup. */
  TEST_ASSERT_NULL(
      aj_sr04m_new(CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
                   CONFIG_AJ_SR04M_TRIGGER_BYTE, LOCK_TEST_UART_PORT));
  aj_sr04m_delete(NULL);
  TEST_ASSERT_NULL(aj_sr04m_get_handle(-1));
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  /* Argument checks of read_all(), then the pool-exhausted exit of new(). */
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(NULL, statuses, AJ_SR04M_MAX_SENSORS, &count));
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(distances, NULL, AJ_SR04M_MAX_SENSORS, &count));
  TEST_ASSERT_EQUAL(
      ESP_ERR_INVALID_ARG,
      aj_sr04m_read_all(distances, statuses, AJ_SR04M_MAX_SENSORS, NULL));
  TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE,
                    aj_sr04m_read_all(distances, statuses, 0, &count));
  TEST_ASSERT_NULL(
      aj_sr04m_new(CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
                   CONFIG_AJ_SR04M_TRIGGER_BYTE, LOCK_TEST_UART_PORT));
  TEST_ASSERT_NULL(aj_sr04m_get_handle(AJ_SR04M_MAX_SENSORS));

  /* Reached here with the table still answering: no probe kept the lock. */
  TEST_ASSERT_EQUAL(AJ_SR04M_MAX_SENSORS, aj_sr04m_get_sensor_count());
  TEST_ASSERT_NOT_NULL(aj_sr04m_get_handle(0));
}

/* The mutex is created once, before any task exists, and never destroyed --
 * destroying it in aj_sr04m_deinit() would only move the race it closes. A
 * driver that came back up without a usable lock would hang here. */
TEST_CASE("locking: the table lock survives a deinit/init round trip",
          "[aj_sr04m][locking]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(AJ_SR04M_MAX_SENSORS, aj_sr04m_get_sensor_count());

  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_deinit());
  TEST_ASSERT_EQUAL(0, aj_sr04m_get_sensor_count());

  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());
  TEST_ASSERT_EQUAL(AJ_SR04M_MAX_SENSORS, aj_sr04m_get_sensor_count());
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
}

#if CONFIG_IDF_TARGET_LINUX

/*
 * Two tasks against the table at once.
 *
 * Linux-only on purpose: the POSIX port runs a real scheduler on host
 * threads, so the two tasks below genuinely interleave, while the QEMU
 * matrix exists to cover per-mode wire behaviour and gains nothing from a
 * timing-shaped case.
 *
 * The assertions are invariants that hold under *any* interleaving, so the
 * case cannot fail intermittently: it either reports a broken invariant or
 * passes. What it is really guarding is the shape of the locking rather
 * than a specific corruption -- a self-deadlock (a public entry point
 * calling another one instead of its _locked helper) shows up as a worker
 * that never finishes, which the bounded waits below turn into a clean
 * failure instead of a suite that hangs until the CI job times out.
 */

#define LOCK_CYCLE_ROUNDS 20
#define LOCK_CHURN_ROUNDS 6
#define LOCK_WORKER_TIMEOUT_MS 10000
#define LOCK_WORKER_STACK 8192
#define LOCK_WORKER_PRIO 5

static SemaphoreHandle_t s_workers_done;
static int s_cycle_bad_count;
static int s_churn_new_failures;

/* Walks the table over and over. Both outcomes are legitimate while the
 * other task is between a delete and a create: ESP_OK once at least one
 * sensor triggered, ESP_ERR_INVALID_STATE while the table is momentarily
 * empty. Anything else, or a count outside the table's bounds, means the
 * walk saw the table mid-shift. */
static void lock_cycle_task(void *arg) {
  (void)arg;

  for (int i = 0; i < LOCK_CYCLE_ROUNDS; i++) {
    const esp_err_t err = aj_sr04m_trigger_all();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
      s_cycle_bad_count++;

    const int count = aj_sr04m_get_sensor_count();
    if (count < 0 || count > AJ_SR04M_MAX_SENSORS)
      s_cycle_bad_count++;

    vTaskDelay(1);
  }

  xSemaphoreGive(s_workers_done);
  vTaskDelete(NULL);
}

/* Removes and re-adds the last sensor, which is the sequence that shifts
 * s_sensor_handles under whoever is walking it. Ends on a create, so the
 * table is back to full strength when the case asserts on it. */
static void lock_churn_task(void *arg) {
  (void)arg;

  for (int i = 0; i < LOCK_CHURN_ROUNDS; i++) {
    aj_sr04m_delete(aj_sr04m_get_handle(aj_sr04m_get_sensor_count() - 1));

    if (aj_sr04m_new(CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
                     CONFIG_AJ_SR04M_TRIGGER_BYTE, LOCK_TEST_UART_PORT) == NULL)
      s_churn_new_failures++;

    vTaskDelay(1);
  }

  xSemaphoreGive(s_workers_done);
  vTaskDelete(NULL);
}

TEST_CASE("locking: sensor churn concurrent with a measurement cycle",
          "[aj_sr04m][locking]") {
  mocks_reset();
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_init());

  s_cycle_bad_count = 0;
  s_churn_new_failures = 0;
  s_workers_done = xSemaphoreCreateCounting(2, 0);
  TEST_ASSERT_NOT_NULL(s_workers_done);

  TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(lock_cycle_task, "aj_lock_cycle",
                                        LOCK_WORKER_STACK, NULL,
                                        LOCK_WORKER_PRIO, NULL));
  TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(lock_churn_task, "aj_lock_churn",
                                        LOCK_WORKER_STACK, NULL,
                                        LOCK_WORKER_PRIO, NULL));

  for (int i = 0; i < 2; i++) {
    TEST_ASSERT_EQUAL_MESSAGE(
        pdTRUE,
        xSemaphoreTake(s_workers_done, pdMS_TO_TICKS(LOCK_WORKER_TIMEOUT_MS)),
        "a worker never finished: the table lock was not released");
  }

  vSemaphoreDelete(s_workers_done);
  s_workers_done = NULL;

  TEST_ASSERT_EQUAL_MESSAGE(0, s_cycle_bad_count,
                            "trigger_all() observed an inconsistent table");
  TEST_ASSERT_EQUAL_MESSAGE(0, s_churn_new_failures,
                            "a slot freed by delete() was not reusable");

  /* The churn task ended on a create, so the table is whole and usable. */
  TEST_ASSERT_EQUAL(AJ_SR04M_MAX_SENSORS, aj_sr04m_get_sensor_count());
  for (int i = 0; i < AJ_SR04M_MAX_SENSORS; i++) {
    TEST_ASSERT_NOT_NULL(aj_sr04m_get_handle(i));
  }
  TEST_ASSERT_EQUAL(ESP_OK, aj_sr04m_trigger_all());
}

#endif /* CONFIG_IDF_TARGET_LINUX */
