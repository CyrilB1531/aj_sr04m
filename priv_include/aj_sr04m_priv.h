/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "driver/rmt_rx.h"
#include "freertos/semphr.h"

#include "aj_sr04m_sw_uart.h"

#if AJ_SR04M_MODE >= 3
#include "driver/uart.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Internal handle structure for a single AJ-SR04M sensor instance.
 *
 * This structure encapsulates all per-sensor state (GPIO pins, RMT channel,
 * UART port, etc.). Each sensor can operate independently.
 */
typedef struct {
  // Hardware configuration
  int trigger_pin;      /**< GPIO pin for TRIGGER (mode 1-2) or UART TX (3-5) */
  int echo_pin;         /**< GPIO pin for ECHO (mode 1-2) or UART RX (3-5) */
  uint8_t trigger_byte; /**< UART trigger byte (modes 4-5 only) */

  // RMT RX resources: modes 1-2 (echo capture) and the modes 4-5 software
  // UART backend (frame capture on the echo / module-TX pin)
  rmt_channel_handle_t rx_channel;
  rmt_symbol_word_t *rx_buffer;
  SemaphoreHandle_t rx_done_sem;
  volatile size_t rx_num_symbols;

#if AJ_SR04M_MODE >= 3
  uart_port_t uart_num;            /**< hardware UART port (HW backend) */
  aj_sr04m_uart_backend_t backend; /**< HW or SW UART backend */
#endif

  bool initialized; /**< true if this instance has been successfully created */
} aj_sr04m_sensor_t;

#ifdef __cplusplus
}
#endif
