/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "aj_sr04m_sw_uart.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void aj_sr04m_sw_uart_encode_byte(uint8_t byte, uint8_t levels[10]) {
  levels[0] = 0; /* start bit */
  for (int i = 0; i < 8; i++) {
    levels[1 + i] = (uint8_t)((byte >> i) & 1u); /* LSB first */
  }
  levels[9] = 1; /* stop bit */
}

/* Bit-level decode state machine, fed one bit at a time. */
typedef struct {
  int frame_pos; /* -1 = waiting for start; 0..7 data; 8 = stop */
  uint8_t cur;
  size_t out_len;
} sw_uart_dec_t;

static void sw_uart_step_bit(sw_uart_dec_t *dec, int level, uint8_t *out,
                             size_t out_cap) {
  if (dec->frame_pos < 0) {
    if (level == 0) { /* falling edge = start bit */
      dec->frame_pos = 0;
      dec->cur = 0;
    }
    return; /* idle high otherwise */
  }
  if (dec->frame_pos < 8) {
    if (level) {
      dec->cur |= (uint8_t)(1u << dec->frame_pos);
    }
    dec->frame_pos++;
    return;
  }
  /* frame_pos == 8: stop bit */
  if (level == 1 && dec->out_len < out_cap) {
    out[dec->out_len++] = dec->cur;
  }
  dec->frame_pos = -1; /* drop on framing error or resync after stop */
}

static void sw_uart_feed_run(sw_uart_dec_t *dec, int level,
                             uint16_t duration_us, uint8_t *out,
                             size_t out_cap) {
  if (duration_us == 0) {
    return;
  }
  int nbits =
      (duration_us + AJ_SR04M_SW_UART_BIT_US / 2) / AJ_SR04M_SW_UART_BIT_US;
  for (int b = 0; b < nbits; b++) {
    sw_uart_step_bit(dec, level, out, out_cap);
  }
}

size_t aj_sr04m_sw_uart_decode(const rmt_symbol_word_t *symbols,
                               size_t num_symbols, uint8_t *out,
                               size_t out_cap) {
  sw_uart_dec_t dec = {.frame_pos = -1, .cur = 0, .out_len = 0};
  for (size_t i = 0; i < num_symbols; i++) {
    sw_uart_feed_run(&dec, symbols[i].level0, symbols[i].duration0, out,
                     out_cap);
    sw_uart_feed_run(&dec, symbols[i].level1, symbols[i].duration1, out,
                     out_cap);
  }
  return dec.out_len;
}

aj_sr04m_uart_backend_t aj_sr04m_sw_uart_resolve_backend(int uart_num,
                                                         int hw_uart_count) {
  if (uart_num < 0 || uart_num >= hw_uart_count) {
    return AJ_SR04M_UART_BACKEND_SW;
  }
  return AJ_SR04M_UART_BACKEND_HW;
}

void aj_sr04m_sw_uart_write_byte(int tx_pin, uint8_t byte) {
  uint8_t levels[AJ_SR04M_SW_UART_FRAME_BITS];
  aj_sr04m_sw_uart_encode_byte(byte, levels);

  /* Suspending the scheduler rather than masking interrupts. What can
   * corrupt the frame is another task taking the CPU between two edges, and
   * vTaskSuspendAll() already rules that out; a critical section would also
   * hold every interrupt off for the ~1 ms the frame lasts, starving the
   * WiFi and BLE stacks, other drivers' completions and the tick itself. */
  vTaskSuspendAll();

  /* Each edge is aimed at an absolute offset from the start of the frame
   * instead of chaining relative delays. An ISR that fires mid-frame is then
   * paid for out of the bit it landed in, rather than pushing every
   * remaining edge back: an 8N1 receiver resynchronises only on the start
   * bit, so accumulated lateness is exactly what it cannot absorb by the
   * time the stop bit is sampled. */
  const int64_t frame_start_us = esp_timer_get_time();
  for (int i = 0; i < AJ_SR04M_SW_UART_FRAME_BITS; i++) {
    gpio_set_level((gpio_num_t)tx_pin, levels[i]);
    const int64_t next_edge_us =
        frame_start_us + (int64_t)(i + 1) * AJ_SR04M_SW_UART_BIT_US;
    const int64_t remaining_us = next_edge_us - esp_timer_get_time();
    if (remaining_us > 0) {
      esp_rom_delay_us((uint32_t)remaining_us);
    }
  }
  gpio_set_level((gpio_num_t)tx_pin, 1); /* leave line idle high */
  xTaskResumeAll();
}
