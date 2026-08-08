/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "aj_sr04m.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "aj_sr04m_priv.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_rom_sys.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"

#if AJ_SR04M_MODE >= 3
#include "driver/uart.h"
#endif

#define AJ_SR04M_TAG "AJ-SR04M"

#define AJ_SR04M_DIST_MIN_VALID_MM 200  /* sensor physical limit */
#define AJ_SR04M_DIST_MAX_VALID_MM 4500 /* sensor physical limit */

/* RMT capture parameters: shared by modes 1-2 (echo) and the modes 4-5
 * software UART backend (frame capture). */
#define AJ_SR04M_RMT_RESOLUTION_HZ 1000000U /* 1 MHz -> 1 us per RMT tick */
#define AJ_SR04M_RMT_NUM_SYMBOLS 64
#define AJ_SR04M_RMT_TIMEOUT_MS 50     /* > round-trip time at max range */
#define AJ_SR04M_RMT_IDLE_NS 30000000U /* 30 ms idle threshold */

#define AJ_SR04M_MAX_SENSORS CONFIG_AJ_SR04M_MAX_SENSORS

/* Global state: array of sensor instances */
static aj_sr04m_sensor_t s_sensors[AJ_SR04M_MAX_SENSORS];
static aj_sr04m_handle_t s_sensor_handles[AJ_SR04M_MAX_SENSORS];
static int s_sensor_count = 0;
static bool s_initialized = false;

/* RMT RX done callback: shared by modes 1-2 echo capture and the modes 4-5
 * software UART backend. */
static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_data) {
  BaseType_t hp_task_woken = pdFALSE;
  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)user_data;
  sensor->rx_num_symbols = edata->num_symbols;
  xSemaphoreGiveFromISR(sensor->rx_done_sem, &hp_task_woken);
  return hp_task_woken == pdTRUE;
}

#if AJ_SR04M_MODE < 3
static uint32_t extract_high_pulse_us(const aj_sr04m_sensor_t *sensor) {
  for (size_t i = 0; i < sensor->rx_num_symbols; i++) {
    const rmt_symbol_word_t *sym = &sensor->rx_buffer[i];
    if (sym->level0 == 1 && sym->duration0 > 0)
      return sym->duration0;
    if (sym->level1 == 1 && sym->duration1 > 0)
      return sym->duration1;
  }
  return 0;
}
#endif

#if AJ_SR04M_MODE >= 3
#define AJ_SR04M_BUFFER_SIZE 128
#define AJ_SR04M_SENSOR_1_UART_PORT CONFIG_AJ_SR04M_UART_NUM
#define AJ_SR04M_SENSOR_2_UART_PORT CONFIG_AJ_SR04M_SENSOR_2_UART_NUM
#define AJ_SR04M_SENSOR_3_UART_PORT CONFIG_AJ_SR04M_SENSOR_3_UART_NUM
#define AJ_SR04M_SENSOR_4_UART_PORT CONFIG_AJ_SR04M_SENSOR_4_UART_NUM
#else
/* GPIO modes (1-2) ignore the UART port; these placeholders keep the
 * shared aj_sr04m_new() signature usable without the UART Kconfig
 * symbols (which only exist in modes 3-5). */
#define AJ_SR04M_SENSOR_1_UART_PORT 0
#define AJ_SR04M_SENSOR_2_UART_PORT 0
#define AJ_SR04M_SENSOR_3_UART_PORT 0
#define AJ_SR04M_SENSOR_4_UART_PORT 0
#endif

aj_sr04m_dist_status_t aj_sr04m_parse_binary_frame(const uint8_t *data, int len,
                                                   int16_t *distance) {
  if (len != 4 || data[0] != 0xFF)
    return AJ_SR04M_DIST_BAD_FRAME;

  uint8_t checksum = (data[0] + data[1] + data[2]) & 0xFF;
  if (checksum != data[3])
    return AJ_SR04M_DIST_BAD_CHECKSUM;

  int16_t mm = ((int16_t)data[1] << 8) | (int16_t)data[2];
  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = mm;
  return AJ_SR04M_DIST_OK;
}

aj_sr04m_dist_status_t aj_sr04m_parse_ascii_frame(const char *data,
                                                  int16_t *distance) {
  if (data == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  const char *p = strstr(data, "Gap=");
  if (p == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  unsigned int mm = 0;
  if (sscanf(p, "Gap=%u mm", &mm) != 1)
    return AJ_SR04M_DIST_BAD_FRAME;

  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = (int16_t)mm;
  return AJ_SR04M_DIST_OK;
}

static void aj_sr04m_register_handle(aj_sr04m_handle_t handle) {
  if (s_sensor_count < AJ_SR04M_MAX_SENSORS) {
    s_sensor_handles[s_sensor_count++] = handle;
  }
}

/* s_sensor_count never exceeds AJ_SR04M_MAX_SENSORS: only
 * aj_sr04m_register_handle() grows it, under that bound. Every loop below
 * still repeats the bound explicitly, so the limit holds locally instead of
 * resting on an invariant established in another function. */
static int aj_sr04m_registered_count(void) {
  return s_sensor_count < AJ_SR04M_MAX_SENSORS ? s_sensor_count
                                               : AJ_SR04M_MAX_SENSORS;
}

static void aj_sr04m_unregister_handle(const struct aj_sr04m_sensor *handle) {
  const int count = aj_sr04m_registered_count();

  int index = -1;
  for (int i = 0; i < count; i++) {
    if (s_sensor_handles[i] == handle) {
      index = i;
      break;
    }
  }

  if (index < 0)
    return;

  for (int i = index; i + 1 < count; i++) {
    s_sensor_handles[i] = s_sensor_handles[i + 1];
  }
  s_sensor_handles[count - 1] = NULL;
  s_sensor_count--;
}

static void aj_sr04m_cleanup_configured_sensors(void) {
  while (s_sensor_count > 0) {
    aj_sr04m_delete(s_sensor_handles[0]);
  }
}

static esp_err_t aj_sr04m_configure_sensors_from_kconfig(void) {
  const struct aj_sr04m_sensor *handle =
      aj_sr04m_new(CONFIG_AJ_SR04M_TRIGGER_PIN, CONFIG_AJ_SR04M_ECHO_PIN,
                   CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_1_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;

#if AJ_SR04M_MAX_SENSORS >= 2
  handle = aj_sr04m_new(
      CONFIG_AJ_SR04M_SENSOR_2_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_2_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_2_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

#if AJ_SR04M_MAX_SENSORS >= 3
  handle = aj_sr04m_new(
      CONFIG_AJ_SR04M_SENSOR_3_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_3_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_3_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

#if AJ_SR04M_MAX_SENSORS >= 4
  handle = aj_sr04m_new(
      CONFIG_AJ_SR04M_SENSOR_4_TRIGGER_PIN, CONFIG_AJ_SR04M_SENSOR_4_ECHO_PIN,
      CONFIG_AJ_SR04M_TRIGGER_BYTE, AJ_SR04M_SENSOR_4_UART_PORT);
  if (handle == NULL)
    return ESP_ERR_INVALID_STATE;
#endif

  return ESP_OK;
}

esp_err_t aj_sr04m_init(void) {
  if (s_initialized)
    return ESP_OK;

  memset(s_sensors, 0, sizeof(s_sensors));
  memset(s_sensor_handles, 0, sizeof(s_sensor_handles));
  s_sensor_count = 0;
  s_initialized = true;

  esp_err_t err = aj_sr04m_configure_sensors_from_kconfig();
  if (err != ESP_OK) {
    aj_sr04m_cleanup_configured_sensors();
    s_initialized = false;
    return err;
  }

#if AJ_SR04M_MODE >= 3
  /* UART modes: global driver setup if needed (per-sensor UART config
   * happens in aj_sr04m_new) */
#endif

  return ESP_OK;
}

esp_err_t aj_sr04m_deinit(void) {
  aj_sr04m_cleanup_configured_sensors();
  memset(s_sensors, 0, sizeof(s_sensors));
  memset(s_sensor_handles, 0, sizeof(s_sensor_handles));
  s_sensor_count = 0;
  s_initialized = false;
  return ESP_OK;
}

aj_sr04m_handle_t aj_sr04m_new(int trigger_pin, int echo_pin,
                               uint8_t trigger_byte, int uart_num) {
  if (!s_initialized) {
    ESP_LOGE(AJ_SR04M_TAG,
             "Driver not initialized. Call aj_sr04m_init() first.");
    return NULL;
  }

  /* Find a free slot */
  aj_sr04m_sensor_t *sensor = NULL;
  for (int i = 0; i < AJ_SR04M_MAX_SENSORS; i++) {
    if (!s_sensors[i].initialized) {
      sensor = &s_sensors[i];
      break;
    }
  }

  if (sensor == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Max sensors (%d) reached", AJ_SR04M_MAX_SENSORS);
    return NULL;
  }

  /* Initialize the sensor structure */
  sensor->trigger_pin = trigger_pin;
  sensor->echo_pin = echo_pin;
  sensor->trigger_byte = trigger_byte;

#if AJ_SR04M_MODE < 3
  /* GPIO + RMT mode setup */
  gpio_config_t trigger_cfg = {
      .pin_bit_mask = 1ULL << trigger_pin,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  if (gpio_config(&trigger_cfg) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to configure trigger pin %d", trigger_pin);
    return NULL;
  }

  if (gpio_set_level(trigger_pin, 0) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to set trigger pin %d low", trigger_pin);
    return NULL;
  }

  /* Allocate RMT RX buffer */
  sensor->rx_buffer =
      malloc(AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t));
  if (sensor->rx_buffer == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate RMT RX buffer");
    return NULL;
  }

  /* Configure and allocate RMT RX channel */
  rmt_rx_channel_config_t rx_chan_cfg = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = AJ_SR04M_RMT_RESOLUTION_HZ,
      .mem_block_symbols = AJ_SR04M_RMT_NUM_SYMBOLS,
      .gpio_num = echo_pin,
  };
  if (rmt_new_rx_channel(&rx_chan_cfg, &sensor->rx_channel) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate RMT RX channel for pin %d",
             echo_pin);
    free(sensor->rx_buffer);
    return NULL;
  }

  /* Create semaphore for RMT done callback */
  sensor->rx_done_sem = xSemaphoreCreateBinary();
  if (sensor->rx_done_sem == NULL) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to create RMT semaphore");
    rmt_del_channel(sensor->rx_channel);
    free(sensor->rx_buffer);
    return NULL;
  }

  /* Register RMT callback */
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_cb,
  };
  if (rmt_rx_register_event_callbacks(sensor->rx_channel, &cbs, sensor) !=
      ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to register RMT callback");
    vSemaphoreDelete(sensor->rx_done_sem);
    rmt_del_channel(sensor->rx_channel);
    free(sensor->rx_buffer);
    return NULL;
  }

  /* Enable RMT channel */
  if (rmt_enable(sensor->rx_channel) != ESP_OK) {
    ESP_LOGE(AJ_SR04M_TAG, "Unable to enable RMT channel");
    vSemaphoreDelete(sensor->rx_done_sem);
    rmt_del_channel(sensor->rx_channel);
    free(sensor->rx_buffer);
    return NULL;
  }

#else
  /* UART mode setup: resolve hardware vs software (RMT) backend. */
  sensor->backend = aj_sr04m_sw_uart_resolve_backend(uart_num, SOC_UART_NUM);

  if (sensor->backend == AJ_SR04M_UART_BACKEND_HW) {
    const int uart_buffer_size = (AJ_SR04M_BUFFER_SIZE * 2);
    sensor->uart_num = (uart_port_t)uart_num;

    if (uart_driver_install(sensor->uart_num, uart_buffer_size, 0, 10, NULL,
                            0) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to install UART driver on port %d",
               sensor->uart_num);
      return NULL;
    }

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(sensor->uart_num, &uart_config) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to configure UART %d", sensor->uart_num);
      uart_driver_delete(sensor->uart_num);
      return NULL;
    }

    if (uart_set_pin(sensor->uart_num, trigger_pin, echo_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to set UART pins");
      uart_driver_delete(sensor->uart_num);
      return NULL;
    }
    ESP_LOGI(AJ_SR04M_TAG, "Sensor UART(HW port %d): tx=%d rx=%d",
             sensor->uart_num, trigger_pin, echo_pin);
  } else {
    /* Software UART: GPIO TX (idle high) + RMT RX on the echo pin. */
    gpio_config_t tx_cfg = {
        .pin_bit_mask = 1ULL << trigger_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&tx_cfg) != ESP_OK ||
        gpio_set_level(trigger_pin, 1) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to configure SW UART TX pin %d",
               trigger_pin);
      return NULL;
    }

    sensor->rx_buffer =
        malloc(AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t));
    if (sensor->rx_buffer == NULL) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate SW UART RX buffer");
      return NULL;
    }

    rmt_rx_channel_config_t rx_chan_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = AJ_SR04M_RMT_RESOLUTION_HZ,
        .mem_block_symbols = AJ_SR04M_RMT_NUM_SYMBOLS,
        .gpio_num = echo_pin,
    };
    if (rmt_new_rx_channel(&rx_chan_cfg, &sensor->rx_channel) != ESP_OK) {
      ESP_LOGE(AJ_SR04M_TAG, "Unable to allocate SW UART RMT channel");
      free(sensor->rx_buffer);
      return NULL;
    }

    sensor->rx_done_sem = xSemaphoreCreateBinary();
    if (sensor->rx_done_sem == NULL) {
      rmt_del_channel(sensor->rx_channel);
      free(sensor->rx_buffer);
      return NULL;
    }

    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rmt_rx_done_cb};
    if (rmt_rx_register_event_callbacks(sensor->rx_channel, &cbs, sensor) !=
            ESP_OK ||
        rmt_enable(sensor->rx_channel) != ESP_OK) {
      vSemaphoreDelete(sensor->rx_done_sem);
      rmt_del_channel(sensor->rx_channel);
      free(sensor->rx_buffer);
      return NULL;
    }
    ESP_LOGI(AJ_SR04M_TAG, "Sensor UART(SW/RMT): tx=%d rx=%d", trigger_pin,
             echo_pin);
  }

#endif

  sensor->initialized = true;
  aj_sr04m_register_handle((aj_sr04m_handle_t)sensor);
  ESP_LOGI(AJ_SR04M_TAG, "Sensor created: trigger=%d, echo=%d", trigger_pin,
           echo_pin);

  return (aj_sr04m_handle_t)sensor;
}

void aj_sr04m_delete(aj_sr04m_handle_t handle) {
  if (handle == NULL)
    return;

  aj_sr04m_unregister_handle(handle);

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return;

#if AJ_SR04M_MODE < 3
  /* GPIO + RMT cleanup */
  if (sensor->rx_channel != NULL) {
    rmt_disable(sensor->rx_channel);
    rmt_del_channel(sensor->rx_channel);
  }
  if (sensor->rx_done_sem != NULL) {
    vSemaphoreDelete(sensor->rx_done_sem);
  }
  if (sensor->rx_buffer != NULL) {
    free(sensor->rx_buffer);
  }
#else
  /* SW backend owns RMT/GPIO resources; HW backend owns a UART driver. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    if (sensor->rx_channel != NULL) {
      rmt_disable(sensor->rx_channel);
      rmt_del_channel(sensor->rx_channel);
    }
    if (sensor->rx_done_sem != NULL) {
      vSemaphoreDelete(sensor->rx_done_sem);
    }
    if (sensor->rx_buffer != NULL) {
      free(sensor->rx_buffer);
    }
  } else {
    uart_driver_delete(sensor->uart_num);
  }
#endif

  sensor->initialized = false;
  memset(sensor, 0, sizeof(*sensor));
}

int aj_sr04m_get_sensor_count(void) { return s_sensor_count; }

esp_err_t aj_sr04m_trigger_all(void) {
  if (!s_initialized || s_sensor_count == 0)
    return ESP_ERR_INVALID_STATE;

  const int count = aj_sr04m_registered_count();
  for (int i = 0; i < count; i++) {
    /* Co-located modules hear each other's 40 kHz burst. Firing them back
     * to back makes the neighbour's burst race the real echo, and the
     * loser reports its out-of-range sentinel. Spacing the triggers keeps
     * each measurement window clear. */
    if (i > 0 && AJ_SR04M_TRIGGER_STAGGER_MS > 0)
      vTaskDelay(pdMS_TO_TICKS(AJ_SR04M_TRIGGER_STAGGER_MS));
    aj_sr04m_trigger(s_sensor_handles[i]);
  }

  return ESP_OK;
}

esp_err_t aj_sr04m_read_all(int16_t *distances,
                            aj_sr04m_dist_status_t *statuses, int max_sensors,
                            int *out_sensor_count) {
  if (distances == NULL || statuses == NULL || out_sensor_count == NULL)
    return ESP_ERR_INVALID_ARG;
  if (!s_initialized || s_sensor_count == 0)
    return ESP_ERR_INVALID_STATE;
  if (max_sensors < s_sensor_count)
    return ESP_ERR_INVALID_SIZE;

  const int count = aj_sr04m_registered_count();
  for (int i = 0; i < count; i++) {
    statuses[i] = aj_sr04m_read_distance(s_sensor_handles[i], &distances[i]);
  }

  *out_sensor_count = count;
  return ESP_OK;
}

void aj_sr04m_trigger(aj_sr04m_handle_t handle) {
  if (handle == NULL)
    return;

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return;

#if AJ_SR04M_MODE < 3
  rmt_receive_config_t rx_cfg = {
      .signal_range_min_ns = 1000, /* filter glitches < 1 us */
      .signal_range_max_ns =
          AJ_SR04M_RMT_IDLE_NS, /* idle threshold = end of frame */
  };
  rmt_receive(sensor->rx_channel, sensor->rx_buffer,
              AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t), &rx_cfg);

  gpio_set_level(sensor->trigger_pin, 1);
  esp_rom_delay_us(
#if AJ_SR04M_MODE == 1
      15
#else
      1100
#endif
  );
  gpio_set_level(sensor->trigger_pin, 0);
#elif AJ_SR04M_MODE >= 4
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = AJ_SR04M_RMT_IDLE_NS,
    };
    rmt_receive(sensor->rx_channel, sensor->rx_buffer,
                AJ_SR04M_RMT_NUM_SYMBOLS * sizeof(rmt_symbol_word_t), &rx_cfg);
    aj_sr04m_sw_uart_write_byte(sensor->trigger_pin, sensor->trigger_byte);
  } else {
    uart_write_bytes(sensor->uart_num, &sensor->trigger_byte, 1);
  }
#endif
}

aj_sr04m_dist_status_t aj_sr04m_read_distance(aj_sr04m_handle_t handle,
                                              int16_t *distance) {
  if (handle == NULL || distance == NULL)
    return AJ_SR04M_DIST_BAD_FRAME;

  aj_sr04m_sensor_t *sensor = (aj_sr04m_sensor_t *)handle;

  if (!sensor->initialized)
    return AJ_SR04M_DIST_BAD_FRAME;

#if AJ_SR04M_MODE <= 2
  if (xSemaphoreTake(sensor->rx_done_sem,
                     pdMS_TO_TICKS(AJ_SR04M_RMT_TIMEOUT_MS)) != pdTRUE)
    return AJ_SR04M_DIST_NO_ECHO;

  uint32_t pulse_us = extract_high_pulse_us(sensor);
  if (pulse_us == 0)
    return AJ_SR04M_DIST_NO_ECHO;

  int16_t mm = (int16_t)((float)pulse_us * 0.1715f);
  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = mm;
  return AJ_SR04M_DIST_OK;
#else
  /* Modes 3-5. Software backend: wait for the RMT capture, decode the 9600
   * 8N1 bytes, then reuse the same frame parsers as the hardware backend. */
  if (sensor->backend == AJ_SR04M_UART_BACKEND_SW) {
    if (xSemaphoreTake(sensor->rx_done_sem,
                       pdMS_TO_TICKS(AJ_SR04M_RMT_TIMEOUT_MS)) != pdTRUE)
      return AJ_SR04M_DIST_NO_ECHO;
    uint8_t bytes[64];
    size_t n = aj_sr04m_sw_uart_decode(
        sensor->rx_buffer, sensor->rx_num_symbols, bytes, sizeof(bytes));
#if AJ_SR04M_MODE == 5
    bytes[n < sizeof(bytes) ? n : sizeof(bytes) - 1] = '\0';
    return aj_sr04m_parse_ascii_frame((const char *)bytes, distance);
#else
    return aj_sr04m_parse_binary_frame(bytes, (int)n, distance);
#endif
  }

#if AJ_SR04M_MODE == 5
  /* ASCII frame "Gap=XXXX mm\r\n", reply latency ~100-200 ms */
  char data[64];
  int len = uart_read_bytes(sensor->uart_num, (uint8_t *)data, sizeof(data) - 1,
                            pdMS_TO_TICKS(250));
  if (len <= 0)
    return AJ_SR04M_DIST_BAD_FRAME;
  data[len] = '\0';
  return aj_sr04m_parse_ascii_frame(data, distance);
#else
  /* Binary frame: 0xFF | dist_H | dist_L | checksum */
  uint8_t data[16];
  int len =
      uart_read_bytes(sensor->uart_num, data, 15, 20 / portTICK_PERIOD_MS);
  return aj_sr04m_parse_binary_frame(data, len, distance);
#endif
#endif
}
