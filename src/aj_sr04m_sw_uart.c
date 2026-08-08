/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "aj_sr04m_sw_uart.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

static portMUX_TYPE s_sw_uart_mux = portMUX_INITIALIZER_UNLOCKED;

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

  portENTER_CRITICAL(&s_sw_uart_mux);
  for (int i = 0; i < AJ_SR04M_SW_UART_FRAME_BITS; i++) {
    gpio_set_level((gpio_num_t)tx_pin, levels[i]);
    esp_rom_delay_us(AJ_SR04M_SW_UART_BIT_US);
  }
  gpio_set_level((gpio_num_t)tx_pin, 1); /* leave line idle high */
  portEXIT_CRITICAL(&s_sw_uart_mux);
}
