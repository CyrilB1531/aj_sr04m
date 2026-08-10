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

#include "aj_sr04m.h"

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#endif

/* State recorded by the __wrap_uart_* mocks in mocks.c. Tests reset it via
 * mocks_reset() before exercising the code under test, then assert on the
 * recorded calls. */
struct uart_mock_state {
  int driver_install_calls;
  int driver_delete_calls;
  int param_config_calls;
  int set_pin_calls;
  int write_bytes_calls;
  int read_bytes_calls;
  int flush_input_calls;
  /* Value of flush_input_calls when the last trigger byte went out. Lets a
   * test assert the flush happened *before* the write rather than merely
   * somewhere in the cycle. */
  int flush_calls_at_write;

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

#if CONFIG_IDF_TARGET_LINUX || CONFIG_AJ_SR04M_MODE_1 || CONFIG_AJ_SR04M_MODE_2

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
  /* Direction of the most recent gpio_config(). Tests assert on
   * GPIO_MODE_INPUT to check that a deleted or failed sensor stopped
   * driving its trigger pin. */
  int last_mode;

/* Every level driven, in order, so a test can read back the waveform a
 * bit-banged UART frame put on the wire instead of only its last edge.
 * Recording stops once full — a saturated buffer shows up as a length
 * mismatch rather than as wrapped-around garbage. */
#define GPIO_MOCK_MAX_LEVELS 32
  int level_seq[GPIO_MOCK_MAX_LEVELS];
  int level_seq_len;
};

extern struct gpio_mock_state g_gpio_mock;

/* `esp_rom_delay_us` mock — records the pulse-width argument so tests
 * verify mode 1 (15 µs) vs mode 2 (1100 µs) trigger duration. */
struct esp_rom_mock_state {
  int delay_us_calls;
  uint32_t last_delay_us;

/* Every requested delay, in order, so a test can check how a bit-banged
 * frame re-planned its edges rather than only how long the last one was. */
#define ESP_ROM_MOCK_MAX_DELAYS 32
  uint32_t delay_seq[ESP_ROM_MOCK_MAX_DELAYS];
  int delay_seq_len;

  /* Virtual microsecond clock, advanced by every delay served. On the linux
   * target it also backs esp_timer_get_time(), which has no implementation
   * there — so code that busy-waits then reads the clock sees time move the
   * way it would on the chip, deterministically. */
  int64_t now_us;

  /* One-shot overrun injection: `overrun_us` extra microseconds are charged
   * to the virtual clock on delay call number `overrun_at_call` (0-based),
   * standing in for an ISR that ran while the frame was in flight. Inert
   * while overrun_us is 0. */
  uint32_t overrun_us;
  int overrun_at_call;
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

  /* Idle thresholds the driver asked for, straight from the last
   * rmt_receive_config_t. signal_range_max_ns is what ends a capture, so it
   * is the driver's statement of how long a level run may last — the echo
   * pulse included. */
  uint32_t last_signal_range_min_ns;
  uint32_t last_signal_range_max_ns;

  bool fire_pulse_on_receive;
  uint32_t fire_pulse_high_us;

  /* Delay, in milliseconds, between the arming and the completion. Zero
   * fires synchronously inside rmt_receive, which is what most cases want;
   * a non-zero value defers the callback to a helper task, so the read
   * really blocks and its timeout is exercised. One deferred capture at a
   * time — the helper writes into the buffer of the receive that armed it,
   * so a case must let the completion land before the sensor is deleted. */
  uint32_t fire_pulse_delay_ms;

  /* Bytes to synthesise as a 9600 8N1 line capture instead of the single
   * echo pulse, for the software UART backend. The mock encodes them the way
   * the module would drive the wire — start bit, 8 data bits LSB first, stop
   * bit — and merges identical neighbouring levels into one run, as the RMT
   * hardware does. Points at caller-owned memory that must outlive the
   * receive. */
  const uint8_t *fire_uart_bytes;
  size_t fire_uart_len;

  /* Report the completion as filling the whole buffer, which is how a
   * truncated capture reaches the driver: the RMT engine stops at the end of
   * its memory and hands back everything it stored. */
  bool fire_capture_fills_buffer;

  /* The driver registers the same function for every sensor; the context
   * that tells them apart is kept per channel inside mocks.c. */
  aj_sr04m_rmt_rx_done_cb_t on_recv_done;
};

extern struct rmt_mock_state g_rmt_mock;

/* Teardown steps, recorded in the order the driver performs them. Counters
 * cannot express what matters when a sensor is released: the channel has to
 * stop delivering completions before the semaphore its ISR callback gives is
 * destroyed, and both happening is not the same as them happening in that
 * order. MOCKS_TEARDOWN_SEM_DELETE is only ever recorded on the linux
 * target, where the queue wraps live. */
typedef enum {
  MOCKS_TEARDOWN_RMT_DISABLE,
  MOCKS_TEARDOWN_RMT_DEL_CHANNEL,
  MOCKS_TEARDOWN_SEM_DELETE,
} mocks_teardown_step_t;

#define MOCKS_TEARDOWN_MAX_STEPS 32

struct teardown_mock_state {
  mocks_teardown_step_t steps[MOCKS_TEARDOWN_MAX_STEPS];
  int steps_len;
};

extern struct teardown_mock_state g_teardown_mock;

/**
 * @brief Position of a teardown step in the recorded sequence.
 *
 * @param step step to look for
 *
 * @return
 *    - the index of its first occurrence
 *    - -1 if it was never recorded
 */
int mocks_teardown_step_index(mocks_teardown_step_t step);

#endif /* hardware-driver mocks (linux all modes, ESP modes 1-2) */

#if CONFIG_IDF_TARGET_LINUX

/* Allocation-failure injection, for the aj_sr04m_new() exits that no driver
 * mock can reach: the RMT buffer malloc and the rx_done_sem creation.
 *
 * Both wraps stay inert until a flag is armed, and each targets its call
 * narrowly — malloc only fails for a buffer of exactly the driver's RMT
 * capture size, and the queue wrap only for a binary semaphore. That is what
 * keeps them from disturbing the allocations ESP-IDF, Unity and the C
 * library make around the code under test. Each flag disarms itself once it
 * has fired, so one armed flag injects exactly one failure. */
struct heap_mock_state {
  bool fail_rmt_buffer_alloc; /**< next RMT-sized malloc returns NULL */
  bool fail_semaphore_create; /**< next binary semaphore returns NULL */
  int rmt_buffer_alloc_calls; /**< RMT-sized mallocs seen */
  int semaphore_create_calls; /**< binary semaphores created */
};

extern struct heap_mock_state g_heap_mock;

#endif /* CONFIG_IDF_TARGET_LINUX */

/* Zero out the mock state and set sensible defaults (success returns). Call
 * at the top of every TEST_CASE that uses the wrapped functions. */
void mocks_reset(void);

/**
 * @brief Read the single configured sensor through the multi-sensor API.
 *
 * Mirrors the old single-sensor read convenience: the caller must have called
 * aj_sr04m_init() first so exactly one sensor is configured.
 *
 * @param[out] dist distance in millimeters (valid only if the return is
 * AJ_SR04M_DIST_OK)
 *
 * @return
 *    - the measurement status of the single configured sensor
 *    - AJ_SR04M_DIST_BAD_FRAME if the read could not be performed
 */
aj_sr04m_dist_status_t mocks_read_one(int16_t *dist);
