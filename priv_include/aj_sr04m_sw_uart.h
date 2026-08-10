/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_rx.h" /* rmt_symbol_word_t */

#ifdef __cplusplus
extern "C" {
#endif

#define AJ_SR04M_SW_UART_BAUD 9600
#define AJ_SR04M_SW_UART_BIT_US 104    /**< 1e6 / 9600 ≈ 104.17 us, rounded */
#define AJ_SR04M_SW_UART_FRAME_BITS 10 /**< 1 start + 8 data + 1 stop */

/**
 * @brief UART backend selected for a sensor in modes 3-5.
 */
typedef enum {
  AJ_SR04M_UART_BACKEND_HW, /**< dedicated hardware UART controller */
  AJ_SR04M_UART_BACKEND_SW, /**< software UART bit-banged/RMT on GPIOs */
} aj_sr04m_uart_backend_t;

/**
 * @brief Encode one byte as the 10 line levels of a 9600 8N1 frame.
 *
 * @param byte        byte to encode
 * @param[out] levels array of 10 levels: index 0 = start bit (0), indices
 *                    1..8 = data bits LSB-first, index 9 = stop (1)
 */
void aj_sr04m_sw_uart_encode_byte(uint8_t byte, uint8_t levels[10]);

/**
 * @brief Decode RMT-captured 9600 8N1 symbols into bytes.
 *
 * The idle line is HIGH; each frame is start(LOW) + 8 data bits LSB-first +
 * stop(HIGH). The bit count of a level run is round(duration_us / 104).
 * Frames with an invalid (LOW) stop bit are dropped.
 *
 * @param symbols     RMT symbols captured at 1 MHz (1 tick = 1 us)
 * @param num_symbols number of valid symbols in @p symbols
 * @param[out] out    buffer receiving decoded bytes
 * @param out_cap     capacity of @p out in bytes
 *
 * @return number of bytes written to @p out (0..out_cap)
 */
size_t aj_sr04m_sw_uart_decode(const rmt_symbol_word_t *symbols,
                               size_t num_symbols, uint8_t *out,
                               size_t out_cap);

/**
 * @brief Resolve the UART backend from a configured port number.
 *
 * @param uart_num      configured port number
 * @param hw_uart_count number of real UART controllers (e.g. SOC_UART_NUM)
 *
 * @return
 *    - AJ_SR04M_UART_BACKEND_HW if uart_num is in [0, hw_uart_count)
 *    - AJ_SR04M_UART_BACKEND_SW otherwise
 */
aj_sr04m_uart_backend_t aj_sr04m_sw_uart_resolve_backend(int uart_num,
                                                         int hw_uart_count);

/**
 * @brief Bit-bang one byte at 9600 8N1 on a GPIO (software UART TX).
 *
 * Drives @p tx_pin with the start bit, 8 data bits LSB-first and the stop
 * bit. Every edge is aimed at an absolute offset from the start of the
 * frame — AJ_SR04M_SW_UART_BIT_US apart — so an interruption is charged to
 * the bit it lands in instead of shifting the rest of the frame.
 *
 * @note The scheduler is suspended for the ~1 ms the frame takes, which
 *       keeps any task from preempting it, but interrupts stay enabled. An
 *       ISR running longer than one bit time can therefore still stretch a
 *       bit beyond what the module's receiver tolerates. That failure is
 *       recoverable: the module does not answer, the read reports no echo,
 *       and the next measurement cycle triggers again.
 *
 * @param tx_pin GPIO already configured as output
 * @param byte   byte to transmit
 */
void aj_sr04m_sw_uart_write_byte(int tx_pin, uint8_t byte);

#ifdef __cplusplus
}
#endif
