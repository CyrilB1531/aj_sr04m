// #define CONFIG_USE_LIGHT_SLEEP 1
#include "aj_sr04m.h"

const char MAIN_TAG[] = "MAIN";

void delay_ms(uint8_t ms) { vTaskDelay(ms / 10); }

void app_main(void) {
  int16_t distance = 0;

  ESP_ERROR_CHECK(aj_sr04m_init());

  while (true) {
    aj_sr04m_trigger();
    aj_sr04m_dist_status_t status = aj_sr04m_read_duration(&distance);
    if (status == AJ_SR04M_DIST_OK) {
      ESP_LOGI(MAIN_TAG, "Distance: %ld", distance);
    } else {
      ESP_LOGW(MAIN_TAG, "Status: %d", status);
    }
    delay_ms(
#if AJ_SR04M_MODE == 3
        1
#else
        2000
#endif
    );
  }
}
