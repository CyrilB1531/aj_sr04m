/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#include "driver/uart.h"

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#endif

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

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2

/* GPIO mock state (modes 1-2 only) — records configure/set_level calls
 * and lets tests inject error returns to exercise ESP_RETURN_ON_ERROR
 * branches. */
struct gpio_mock_state {
  int config_calls;
  int set_level_calls;
  esp_err_t config_ret;
  esp_err_t set_level_ret;
  uint64_t last_pin_bit_mask;
  int last_pin;
  int last_level;
};

extern struct gpio_mock_state g_gpio_mock;

/* `esp_rom_delay_us` mock — records the pulse-width argument so tests
 * verify mode 1 (15 µs) vs mode 2 (1100 µs) trigger duration. */
struct esp_rom_mock_state {
  int delay_us_calls;
  uint32_t last_delay_us;
};

extern struct esp_rom_mock_state g_esp_rom_mock;

/* RMT mock state — captures the receive buffer pointer (so the mock can
 * write a synthetic `rmt_symbol_word_t` to it) and the `on_recv_done`
 * callback (so the mock can fire it synchronously, simulating the
 * hardware "RX done" event). When `fire_pulse_on_receive` is set,
 * __wrap_rmt_receive populates one symbol { level0=1,
 * duration0=fire_pulse_high_us } and invokes the callback before
 * returning. */
typedef bool (*aj_sr04m_rmt_rx_done_cb_t)(rmt_channel_handle_t,
                                          const rmt_rx_done_event_data_t *,
                                          void *);

struct rmt_mock_state {
  int new_rx_channel_calls;
  int register_event_callbacks_calls;
  int enable_calls;
  int receive_calls;

  esp_err_t new_rx_channel_ret;
  esp_err_t register_event_callbacks_ret;
  esp_err_t enable_ret;
  esp_err_t receive_ret;

  void *last_receive_buffer;
  size_t last_receive_buffer_size;

  bool fire_pulse_on_receive;
  uint32_t fire_pulse_high_us;

  aj_sr04m_rmt_rx_done_cb_t on_recv_done;
  void *on_recv_done_user_data;
};

extern struct rmt_mock_state g_rmt_mock;

#endif /* CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2 */

/* Zero out the mock state and set sensible defaults (success returns). Call
 * at the top of every TEST_CASE that uses the wrapped functions. */
void mocks_reset(void);
