/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Software UART pure-function tests — mode-agnostic, no hardware mocks.
 * Runs in every build like test_parser.c.
 */

#include <stdint.h>
#include <string.h>

#include "unity.h"

#include "aj_sr04m_sw_uart.h"

/* === encode ============================================================ */

TEST_CASE("sw_uart encode: 0x01 -> start,1,0..,stop", "[aj_sr04m][sw_uart]") {
  uint8_t levels[10];
  aj_sr04m_sw_uart_encode_byte(0x01, levels);
  uint8_t expected[10] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, levels, 10);
}

TEST_CASE("sw_uart encode: 0xFF all data bits high", "[aj_sr04m][sw_uart]") {
  uint8_t levels[10];
  aj_sr04m_sw_uart_encode_byte(0xFF, levels);
  uint8_t expected[10] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, levels, 10);
}

TEST_CASE("sw_uart encode: 0x00 only stop high", "[aj_sr04m][sw_uart]") {
  uint8_t levels[10];
  aj_sr04m_sw_uart_encode_byte(0x00, levels);
  uint8_t expected[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, levels, 10);
}

/* === decode helpers ==================================================== */

/* Build a continuous 9600 8N1 line from a list of bytes: idle-high lead-in,
 * then start/data/stop per byte, then idle-high tail. Returns level samples
 * (one per bit) into bits[], count via *n. */
static void build_line_bits(const uint8_t *bytes, size_t nbytes, uint8_t *bits,
                            size_t *n) {
  size_t k = 0;
  bits[k++] = 1; /* idle lead-in */
  for (size_t b = 0; b < nbytes; b++) {
    uint8_t levels[10];
    aj_sr04m_sw_uart_encode_byte(bytes[b], levels);
    for (int i = 0; i < 10; i++) {
      bits[k++] = levels[i];
    }
  }
  bits[k++] = 1; /* idle tail */
  *n = k;
}

/* Pack a per-bit level stream into RMT symbols. Each run of identical levels
 * becomes one half-symbol of duration (run_len * AJ_SR04M_SW_UART_BIT_US).
 * Two halves per rmt_symbol_word_t. Returns the number of symbols used. */
static size_t pack_symbols(const uint8_t *bits, size_t n,
                           rmt_symbol_word_t *syms, size_t cap) {
  size_t half = 0;
  size_t i = 0;
  memset(syms, 0, cap * sizeof(rmt_symbol_word_t));
  while (i < n) {
    size_t run = 1;
    while (i + run < n && bits[i + run] == bits[i]) {
      run++;
    }
    size_t sidx = half / 2;
    if (sidx >= cap) {
      break;
    }
    uint16_t dur = (uint16_t)(run * AJ_SR04M_SW_UART_BIT_US);
    if (half % 2 == 0) {
      syms[sidx].level0 = bits[i];
      syms[sidx].duration0 = dur;
    } else {
      syms[sidx].level1 = bits[i];
      syms[sidx].duration1 = dur;
    }
    half++;
    i += run;
  }
  return (half + 1) / 2;
}

/* === decode ============================================================ */

TEST_CASE("sw_uart decode: recovers a binary frame", "[aj_sr04m][sw_uart]") {
  const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  uint8_t bits[128];
  size_t nbits = 0;
  build_line_bits(frame, 4, bits, &nbits);

  rmt_symbol_word_t syms[64];
  size_t nsyms = pack_symbols(bits, nbits, syms, 64);

  uint8_t out[16];
  size_t got = aj_sr04m_sw_uart_decode(syms, nsyms, out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(4, got);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, out, 4);
}

TEST_CASE("sw_uart decode: recovers an ASCII frame", "[aj_sr04m][sw_uart]") {
  const char *txt = "Gap=1500 mm\r\n";
  size_t len = strlen(txt);
  uint8_t bits[512];
  size_t nbits = 0;
  build_line_bits((const uint8_t *)txt, len, bits, &nbits);

  rmt_symbol_word_t syms[256];
  size_t nsyms = pack_symbols(bits, nbits, syms, 256);

  uint8_t out[32];
  size_t got = aj_sr04m_sw_uart_decode(syms, nsyms, out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(len, got);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(txt, out, len);
}

TEST_CASE("sw_uart decode: respects out_cap", "[aj_sr04m][sw_uart]") {
  const uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  uint8_t bits[128];
  size_t nbits = 0;
  build_line_bits(frame, 4, bits, &nbits);
  rmt_symbol_word_t syms[64];
  size_t nsyms = pack_symbols(bits, nbits, syms, 64);

  uint8_t out[2];
  size_t got = aj_sr04m_sw_uart_decode(syms, nsyms, out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(2, got);
  TEST_ASSERT_EQUAL_UINT8(0xFF, out[0]);
  TEST_ASSERT_EQUAL_UINT8(0x05, out[1]);
}

TEST_CASE("sw_uart decode: rounds noisy bit durations", "[aj_sr04m][sw_uart]") {
  /* One byte 0xDC with each bit duration jittered +20 us; decode must still
   * round to the right bit count. */
  const uint8_t frame[1] = {0xDC};
  uint8_t bits[64];
  size_t nbits = 0;
  build_line_bits(frame, 1, bits, &nbits);
  rmt_symbol_word_t syms[32];
  size_t nsyms = pack_symbols(bits, nbits, syms, 32);
  for (size_t s = 0; s < nsyms; s++) {
    if (syms[s].duration0) {
      syms[s].duration0 = (uint16_t)(syms[s].duration0 + 20);
    }
    if (syms[s].duration1) {
      syms[s].duration1 = (uint16_t)(syms[s].duration1 + 20);
    }
  }
  uint8_t out[4];
  size_t got = aj_sr04m_sw_uart_decode(syms, nsyms, out, sizeof(out));
  TEST_ASSERT_EQUAL_size_t(1, got);
  TEST_ASSERT_EQUAL_UINT8(0xDC, out[0]);
}

/* === resolve_backend =================================================== */

TEST_CASE("sw_uart backend: valid port -> HW", "[aj_sr04m][sw_uart]") {
  TEST_ASSERT_EQUAL(AJ_SR04M_UART_BACKEND_HW,
                    aj_sr04m_sw_uart_resolve_backend(0, 3));
  TEST_ASSERT_EQUAL(AJ_SR04M_UART_BACKEND_HW,
                    aj_sr04m_sw_uart_resolve_backend(2, 3));
}

TEST_CASE("sw_uart backend: out-of-range port -> SW", "[aj_sr04m][sw_uart]") {
  TEST_ASSERT_EQUAL(AJ_SR04M_UART_BACKEND_SW,
                    aj_sr04m_sw_uart_resolve_backend(3, 3));
  TEST_ASSERT_EQUAL(AJ_SR04M_UART_BACKEND_SW,
                    aj_sr04m_sw_uart_resolve_backend(7, 3));
  TEST_ASSERT_EQUAL(AJ_SR04M_UART_BACKEND_SW,
                    aj_sr04m_sw_uart_resolve_backend(-1, 3));
}
