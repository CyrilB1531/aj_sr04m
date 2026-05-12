/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "mocks.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
#include "esp_attr.h"

/* esp_rom_delay_us exists on both ESP targets and the linux port
 * (components/esp_rom/linux/esp_rom_sys.c provides it), so we passthrough
 * unconditionally. On ESP targets IDF's cpu_start.c calls it from Core 1
 * before flash cache is enabled, so __wrap_esp_rom_delay_us must live in
 * IRAM (IRAM_ATTR is a no-op on linux). */
extern void __real_esp_rom_delay_us(uint32_t us);

/* esp_driver_gpio is empty on the linux target (no sources), so there is
 * no __real_gpio_config / __real_gpio_set_level to call. The wraps below
 * therefore stub-return on linux and passthrough on ESP targets, where
 * IDF code paths beyond aj_sr04m may also drive GPIO. */
#if !CONFIG_IDF_TARGET_LINUX
extern esp_err_t __real_gpio_config(const gpio_config_t *cfg);
extern esp_err_t __real_gpio_set_level(gpio_num_t pin, uint32_t level);
#endif
#endif

struct uart_mock_state g_uart_mock;

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
struct gpio_mock_state g_gpio_mock;
struct esp_rom_mock_state g_esp_rom_mock;
struct rmt_mock_state g_rmt_mock;
#endif

void mocks_reset(void) {
  memset(&g_uart_mock, 0, sizeof(g_uart_mock));
  g_uart_mock.driver_install_ret = ESP_OK;
  g_uart_mock.param_config_ret = ESP_OK;
  g_uart_mock.set_pin_ret = ESP_OK;
  g_uart_mock.write_bytes_ret = 1;
  g_uart_mock.read_bytes_ret = 0;
  g_uart_mock.read_buffer = NULL;
  g_uart_mock.read_buffer_len = 0;

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
  memset(&g_gpio_mock, 0, sizeof(g_gpio_mock));
  g_gpio_mock.config_ret = ESP_OK;
  g_gpio_mock.set_level_ret = ESP_OK;

  memset(&g_esp_rom_mock, 0, sizeof(g_esp_rom_mock));

  memset(&g_rmt_mock, 0, sizeof(g_rmt_mock));
  g_rmt_mock.new_rx_channel_ret = ESP_OK;
  g_rmt_mock.register_event_callbacks_ret = ESP_OK;
  g_rmt_mock.enable_ret = ESP_OK;
  g_rmt_mock.receive_ret = ESP_OK;
#endif
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

#if CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2

esp_err_t __wrap_gpio_config(const gpio_config_t *cfg) {
  g_gpio_mock.config_calls++;
  if (cfg) {
    g_gpio_mock.last_pin_bit_mask = cfg->pin_bit_mask;
  }
  if (g_gpio_mock.config_ret != ESP_OK) {
    return g_gpio_mock.config_ret;
  }
#if CONFIG_IDF_TARGET_LINUX
  return ESP_OK;
#else
  return __real_gpio_config(cfg);
#endif
}

esp_err_t __wrap_gpio_set_level(gpio_num_t pin, uint32_t level) {
  g_gpio_mock.set_level_calls++;
  g_gpio_mock.last_pin = (int)pin;
  g_gpio_mock.last_level = (int)level;
  if (g_gpio_mock.set_level_ret != ESP_OK) {
    return g_gpio_mock.set_level_ret;
  }
#if CONFIG_IDF_TARGET_LINUX
  return ESP_OK;
#else
  return __real_gpio_set_level(pin, level);
#endif
}

/* IRAM_ATTR is mandatory: Core 1's early bring-up path calls this
 * (renamed by --wrap) before flash cache is enabled. Without IRAM
 * placement the fetch faults with LoadProhibited. The real ROM
 * implementation must run so cache/PMU timing is preserved. */
void IRAM_ATTR __wrap_esp_rom_delay_us(uint32_t us) {
  g_esp_rom_mock.delay_us_calls++;
  g_esp_rom_mock.last_delay_us = us;
  __real_esp_rom_delay_us(us);
}

/* Stub channel handle that is non-NULL and unique enough to be checked
 * against in tests. The HAL never dereferences it because every RMT call
 * is intercepted by these wraps. */
static int s_stub_rmt_channel_storage = 0;
#define RMT_MOCK_STUB_HANDLE                                                   \
  ((rmt_channel_handle_t) & s_stub_rmt_channel_storage)

esp_err_t __wrap_rmt_new_rx_channel(const rmt_rx_channel_config_t *cfg,
                                    rmt_channel_handle_t *out) {
  (void)cfg;
  g_rmt_mock.new_rx_channel_calls++;
  if (out) {
    *out = RMT_MOCK_STUB_HANDLE;
  }
  return g_rmt_mock.new_rx_channel_ret;
}

esp_err_t
__wrap_rmt_rx_register_event_callbacks(rmt_channel_handle_t channel,
                                       const rmt_rx_event_callbacks_t *cbs,
                                       void *user_data) {
  (void)channel;
  g_rmt_mock.register_event_callbacks_calls++;
  if (cbs) {
    g_rmt_mock.on_recv_done = (aj_sr04m_rmt_rx_done_cb_t)cbs->on_recv_done;
  }
  g_rmt_mock.on_recv_done_user_data = user_data;
  return g_rmt_mock.register_event_callbacks_ret;
}

esp_err_t __wrap_rmt_enable(rmt_channel_handle_t channel) {
  (void)channel;
  g_rmt_mock.enable_calls++;
  return g_rmt_mock.enable_ret;
}

esp_err_t __wrap_rmt_receive(rmt_channel_handle_t channel, void *buffer,
                             size_t buffer_size,
                             const rmt_receive_config_t *cfg) {
  (void)channel;
  (void)cfg;
  g_rmt_mock.receive_calls++;
  g_rmt_mock.last_receive_buffer = buffer;
  g_rmt_mock.last_receive_buffer_size = buffer_size;

  /* Synthesise an immediate RX-done event with one high-pulse symbol.
   * This stands in for the hardware that would otherwise call back
   * asynchronously when the echo arrives. The callback gives the
   * semaphore that aj_sr04m_read_duration() blocks on, so it returns
   * without waiting for the timeout. */
  if (g_rmt_mock.fire_pulse_on_receive && g_rmt_mock.on_recv_done && buffer &&
      buffer_size >= sizeof(rmt_symbol_word_t)) {
    rmt_symbol_word_t *sym = (rmt_symbol_word_t *)buffer;
    memset(sym, 0, sizeof(*sym));
    sym->level0 = 1;
    sym->duration0 = (uint16_t)g_rmt_mock.fire_pulse_high_us;
    rmt_rx_done_event_data_t evt = {
        .num_symbols = 1,
        .received_symbols = sym,
    };
    g_rmt_mock.on_recv_done(RMT_MOCK_STUB_HANDLE, &evt,
                            g_rmt_mock.on_recv_done_user_data);
  }
  return g_rmt_mock.receive_ret;
}

#endif /* CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2 */
