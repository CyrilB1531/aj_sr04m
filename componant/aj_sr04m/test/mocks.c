/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "mocks.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

struct uart_mock_state g_uart_mock;

void mocks_reset(void) {
  memset(&g_uart_mock, 0, sizeof(g_uart_mock));
  g_uart_mock.driver_install_ret = ESP_OK;
  g_uart_mock.param_config_ret = ESP_OK;
  g_uart_mock.set_pin_ret = ESP_OK;
  g_uart_mock.write_bytes_ret = 1;
  g_uart_mock.read_bytes_ret = 0;
  g_uart_mock.read_buffer = NULL;
  g_uart_mock.read_buffer_len = 0;
}

esp_err_t __wrap_uart_driver_install(uart_port_t port, int rx_buffer_size,
                                     int tx_buffer_size, int queue_size,
                                     void *uart_queue, int intr_alloc_flags) {
  (void)rx_buffer_size;
  (void)tx_buffer_size;
  (void)queue_size;
  (void)uart_queue;
  (void)intr_alloc_flags;
  g_uart_mock.driver_install_calls++;
  g_uart_mock.last_port = port;
  return g_uart_mock.driver_install_ret;
}

esp_err_t __wrap_uart_param_config(uart_port_t port,
                                   const uart_config_t *uart_config) {
  (void)port;
  g_uart_mock.param_config_calls++;
  if (uart_config) {
    g_uart_mock.last_config = *uart_config;
  }
  return g_uart_mock.param_config_ret;
}

esp_err_t __wrap_uart_set_pin(uart_port_t port, int tx_io_num, int rx_io_num,
                              int rts_io_num, int cts_io_num) {
  (void)port;
  (void)rts_io_num;
  (void)cts_io_num;
  g_uart_mock.set_pin_calls++;
  g_uart_mock.last_tx_pin = tx_io_num;
  g_uart_mock.last_rx_pin = rx_io_num;
  return g_uart_mock.set_pin_ret;
}

int __wrap_uart_write_bytes(uart_port_t port, const void *src, size_t size) {
  (void)port;
  g_uart_mock.write_bytes_calls++;
  g_uart_mock.last_write_size = size;
  if (size > 0 && src) {
    g_uart_mock.last_write_byte = ((const uint8_t *)src)[0];
  }
  return g_uart_mock.write_bytes_ret;
}

int __wrap_uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
                           TickType_t ticks_to_wait) {
  (void)port;
  (void)ticks_to_wait;
  g_uart_mock.read_bytes_calls++;
  if (g_uart_mock.read_buffer && g_uart_mock.read_buffer_len > 0 && buf) {
    int n = (int)length < g_uart_mock.read_buffer_len
                ? (int)length
                : g_uart_mock.read_buffer_len;
    memcpy(buf, g_uart_mock.read_buffer, (size_t)n);
    return n;
  }
  return g_uart_mock.read_bytes_ret;
}
