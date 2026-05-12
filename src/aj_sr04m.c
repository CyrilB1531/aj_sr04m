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

#if AJ_SR04M_MODE < 3
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_rom_sys.h"
#include "freertos/semphr.h"
#else
#include "driver/uart.h"
#endif

#define AJ_SR04M_TAG "AJ-SR04M"

#define AJ_SR04M_TRIGGER_PIN CONFIG_AJ_SR04M_TRIGGER_PIN
#define AJ_SR04M_ECHO_PIN CONFIG_AJ_SR04M_ECHO_PIN

#define AJ_SR04M_DIST_MIN_VALID_MM 200  /* sensor physical limit */
#define AJ_SR04M_DIST_MAX_VALID_MM 4500 /* sensor physical limit */

#if AJ_SR04M_MODE < 3
#define AJ_SR04M_RMT_RESOLUTION_HZ 1000000U /* 1 MHz -> 1 us per RMT tick */
#define AJ_SR04M_RMT_NUM_SYMBOLS 64
#define AJ_SR04M_RMT_TIMEOUT_MS 50     /* > round-trip time at max range */
#define AJ_SR04M_RMT_IDLE_NS 30000000U /* 30 ms idle threshold */

static rmt_channel_handle_t s_rx_channel = NULL;
static rmt_symbol_word_t s_rx_buffer[AJ_SR04M_RMT_NUM_SYMBOLS];
static SemaphoreHandle_t s_rx_done_sem = NULL;
static volatile size_t s_rx_num_symbols = 0;

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_data) {
  BaseType_t hp_task_woken = pdFALSE;
  s_rx_num_symbols = edata->num_symbols;
  xSemaphoreGiveFromISR(s_rx_done_sem, &hp_task_woken);
  return hp_task_woken == pdTRUE;
}

static uint32_t extract_high_pulse_us(void) {
  for (size_t i = 0; i < s_rx_num_symbols; i++) {
    const rmt_symbol_word_t *sym = &s_rx_buffer[i];
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
static const uart_port_t s_uart_num = (uart_port_t)CONFIG_AJ_SR04M_UART_NUM;
#endif

#if AJ_SR04M_MODE >= 4
/* Mode 3 is autonomous — the sensor streams frames without prompting, so
 * the trigger byte is only needed for modes 4 and 5. */
static const uint8_t s_trigger_signal = CONFIG_AJ_SR04M_TRIGGER_BYTE;
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

esp_err_t aj_sr04m_init(void) {
#if AJ_SR04M_MODE < 3
  gpio_config_t trigger_cfg = {
      .pin_bit_mask = 1ULL << AJ_SR04M_TRIGGER_PIN,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&trigger_cfg), AJ_SR04M_TAG,
                      "Unable to configure trigger pin");
  ESP_RETURN_ON_ERROR(gpio_set_level(AJ_SR04M_TRIGGER_PIN, 0), AJ_SR04M_TAG,
                      "Unable to set trigger pin low");

  rmt_rx_channel_config_t rx_chan_cfg = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = AJ_SR04M_RMT_RESOLUTION_HZ,
      .mem_block_symbols = AJ_SR04M_RMT_NUM_SYMBOLS,
      .gpio_num = AJ_SR04M_ECHO_PIN,
  };
  ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_chan_cfg, &s_rx_channel),
                      AJ_SR04M_TAG, "Unable to allocate RMT RX channel");

  s_rx_done_sem = xSemaphoreCreateBinary();
  if (s_rx_done_sem == NULL) {
    return ESP_ERR_NO_MEM;
  }

  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_cb,
  };
  ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(s_rx_channel, &cbs, NULL),
                      AJ_SR04M_TAG, "Unable to register RMT RX callback");

  ESP_RETURN_ON_ERROR(rmt_enable(s_rx_channel), AJ_SR04M_TAG,
                      "Unable to enable RMT channel");
#else
  const int uart_buffer_size = (AJ_SR04M_BUFFER_SIZE * 2);

  ESP_RETURN_ON_ERROR(
      uart_driver_install(s_uart_num, uart_buffer_size, 0, 10, NULL, 0),
      AJ_SR04M_TAG, "Unable to install UART driver");
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_RETURN_ON_ERROR(uart_param_config(s_uart_num, &uart_config), AJ_SR04M_TAG,
                      "Unable to configure UART");
  ESP_RETURN_ON_ERROR(uart_set_pin(s_uart_num, AJ_SR04M_TRIGGER_PIN,
                                   AJ_SR04M_ECHO_PIN, UART_PIN_NO_CHANGE,
                                   UART_PIN_NO_CHANGE),
                      AJ_SR04M_TAG, "Unable to set UART pins");
#endif
  return ESP_OK;
}

void aj_sr04m_trigger(void) {
#if AJ_SR04M_MODE < 3
  rmt_receive_config_t rx_cfg = {
      .signal_range_min_ns = 1000, /* filter glitches < 1 us */
      .signal_range_max_ns =
          AJ_SR04M_RMT_IDLE_NS, /* idle threshold = end of frame */
  };
  rmt_receive(s_rx_channel, s_rx_buffer, sizeof(s_rx_buffer), &rx_cfg);

  gpio_set_level(AJ_SR04M_TRIGGER_PIN, 1);
  esp_rom_delay_us(
#if AJ_SR04M_MODE == 1
      15
#else
      1100
#endif
  );
  gpio_set_level(AJ_SR04M_TRIGGER_PIN, 0);
#elif AJ_SR04M_MODE >= 4
  uart_write_bytes(s_uart_num, &s_trigger_signal, 1);
#endif
}

aj_sr04m_dist_status_t aj_sr04m_read_duration(int16_t *distance) {
#if AJ_SR04M_MODE <= 2
  if (xSemaphoreTake(s_rx_done_sem, pdMS_TO_TICKS(AJ_SR04M_RMT_TIMEOUT_MS)) !=
      pdTRUE)
    return AJ_SR04M_DIST_NO_ECHO;

  uint32_t pulse_us = extract_high_pulse_us();
  if (pulse_us == 0)
    return AJ_SR04M_DIST_NO_ECHO;

  int16_t mm = (int16_t)((float)pulse_us * 0.1715f);
  if (mm > AJ_SR04M_DIST_MAX_VALID_MM || mm < AJ_SR04M_DIST_MIN_VALID_MM)
    return AJ_SR04M_DIST_NO_ECHO;

  *distance = mm;
  return AJ_SR04M_DIST_OK;
#elif AJ_SR04M_MODE == 5
  /* ASCII frame "Gap=XXXX mm\r\n", reply latency ~100-200 ms */
  char data[64];
  int len = uart_read_bytes(s_uart_num, (uint8_t *)data, sizeof(data) - 1,
                            pdMS_TO_TICKS(250));
  if (len <= 0)
    return AJ_SR04M_DIST_BAD_FRAME;
  data[len] = '\0';
  return aj_sr04m_parse_ascii_frame(data, distance);
#else
  /* Binary frame: 0xFF | dist_H | dist_L | checksum */
  uint8_t data[16];
  int len = uart_read_bytes(s_uart_num, data, 15, 20 / portTICK_PERIOD_MS);
  return aj_sr04m_parse_binary_frame(data, len, distance);
#endif
}
