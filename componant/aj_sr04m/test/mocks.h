/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

/* State recorded by the __wrap_uart_* mocks in mocks.c. Tests reset it via
 * mocks_reset() before exercising the code under test, then assert on the
 * recorded calls. */
struct uart_mock_state {
  int driver_install_calls;
  int param_config_calls;
  int set_pin_calls;
  int write_bytes_calls;
  int read_bytes_calls;

  esp_err_t driver_install_ret;
  esp_err_t param_config_ret;
  esp_err_t set_pin_ret;
  int write_bytes_ret;
  int read_bytes_ret;

  uart_port_t last_port;
  uart_config_t last_config;
  int last_tx_pin;
  int last_rx_pin;
  uint8_t last_write_byte;
  size_t last_write_size;

  /* When read_buffer is non-NULL, __wrap_uart_read_bytes copies from it into
   * the caller's buffer (up to read_buffer_len bytes) and returns the bytes
   * copied. Otherwise it returns read_bytes_ret unchanged. */
  const uint8_t *read_buffer;
  int read_buffer_len;
};

extern struct uart_mock_state g_uart_mock;

/* Zero out the mock state and set sensible defaults (success returns). Call
 * at the top of every TEST_CASE that uses the wrapped functions. */
void mocks_reset(void);
