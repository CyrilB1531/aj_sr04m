/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "aj_sr04m.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char MAIN_TAG[] = "MAIN";

static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void app_main(void) {
  ESP_ERROR_CHECK(aj_sr04m_init());

  const int sensor_count = aj_sr04m_get_sensor_count();
  ESP_LOGI(MAIN_TAG, "Driver initialized, %d sensor(s) configured",
           sensor_count);

  int16_t distances[AJ_SR04M_MAX_SENSORS] = {0};
  aj_sr04m_dist_status_t statuses[AJ_SR04M_MAX_SENSORS] = {0};

  while (true) {
    /* Mode 3 modules stream on their own; the call is a no-op there. With
     * several sensors it also spaces the triggers, see
     * CONFIG_AJ_SR04M_TRIGGER_STAGGER_MS. */
    ESP_ERROR_CHECK(aj_sr04m_trigger_all());

    /* Leave the module time to answer before reading. */
    delay_ms(100);

    int read_count = 0;
    ESP_ERROR_CHECK(aj_sr04m_read_all(distances, statuses, AJ_SR04M_MAX_SENSORS,
                                      &read_count));

    for (int i = 0; i < read_count; i++) {
      if (statuses[i] == AJ_SR04M_DIST_OK) {
        ESP_LOGI(MAIN_TAG, "Sensor %d: %d mm", i, distances[i]);
      } else {
        ESP_LOGW(MAIN_TAG, "Sensor %d: status %d", i, statuses[i]);
      }
    }

    delay_ms(
#if AJ_SR04M_MODE == 3
        1000
#else
        2000
#endif
    );
  }
}
