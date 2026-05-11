/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Stub `driver/rmt_rx.h` for the ESP-IDF linux target test build.
 *
 * The real esp_driver_rmt is empty on linux. `hal/rmt_types.h` is still
 * available and provides `rmt_clock_source_t` + `rmt_symbol_word_t`, so
 * we reuse those and only declare the rx-specific aggregates and entry
 * points the mocks intercept. `RMT_CLK_SRC_DEFAULT` is patched into the
 * linux build by `tests/linux/shims/uart_caps_shim.h` (force-included
 * via CMake).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal/rmt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rmt_channel_t *rmt_channel_handle_t;

typedef struct {
  size_t num_symbols;
  rmt_symbol_word_t *received_symbols;
  struct {
    uint32_t is_last : 1;
  } flags;
} rmt_rx_done_event_data_t;

typedef bool (*rmt_rx_done_callback_t)(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_ctx);

typedef struct {
  rmt_rx_done_callback_t on_recv_done;
} rmt_rx_event_callbacks_t;

typedef struct {
  rmt_clock_source_t clk_src;
  uint32_t resolution_hz;
  size_t mem_block_symbols;
  int gpio_num;
  struct {
    uint32_t invert_in : 1;
    uint32_t with_dma : 1;
    uint32_t io_loop_back : 1;
  } flags;
} rmt_rx_channel_config_t;

typedef struct {
  uint32_t signal_range_min_ns;
  uint32_t signal_range_max_ns;
  struct {
    uint32_t en_partial_rx : 1;
  } flags;
} rmt_receive_config_t;

esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t *config,
                             rmt_channel_handle_t *ret_chan);
esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t rx_channel,
                                          const rmt_rx_event_callbacks_t *cbs,
                                          void *user_data);
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_receive(rmt_channel_handle_t rx_channel, void *buffer,
                      size_t buffer_size, const rmt_receive_config_t *config);

#ifdef __cplusplus
}
#endif
