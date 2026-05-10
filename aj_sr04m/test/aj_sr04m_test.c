/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>

#include "unity.h"

#include "aj_sr04m.h"

/* === Binary frame parser (modes 3 and 4) =============================== */

TEST_CASE("binary parser: valid frame at 1500 mm", "[aj_sr04m][parser]") {
  /* 1500 = 0x05DC, checksum = (0xFF + 0x05 + 0xDC) & 0xFF = 0xE0 */
  uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0xE0};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("binary parser: lower bound 200 mm", "[aj_sr04m][parser]") {
  /* 200 = 0x00C8, checksum = (0xFF + 0x00 + 0xC8) & 0xFF = 0xC7 */
  uint8_t frame[4] = {0xFF, 0x00, 0xC8, 0xC7};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
  TEST_ASSERT_EQUAL_INT16(200, dist);
}

TEST_CASE("binary parser: upper bound 4500 mm", "[aj_sr04m][parser]") {
  /* 4500 = 0x1194, checksum = (0xFF + 0x11 + 0x94) & 0xFF = 0xA4 */
  uint8_t frame[4] = {0xFF, 0x11, 0x94, 0xA4};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
  TEST_ASSERT_EQUAL_INT16(4500, dist);
}

TEST_CASE("binary parser: below minimum returns NO_ECHO",
          "[aj_sr04m][parser]") {
  /* 199 = 0x00C7, checksum = (0xFF + 0x00 + 0xC7) & 0xFF = 0xC6 */
  uint8_t frame[4] = {0xFF, 0x00, 0xC7, 0xC6};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
}

TEST_CASE("binary parser: 6016 mm sentinel returns NO_ECHO",
          "[aj_sr04m][parser]") {
  /* 6016 = 0x1780, checksum = (0xFF + 0x17 + 0x80) & 0xFF = 0x96 */
  uint8_t frame[4] = {0xFF, 0x17, 0x80, 0x96};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
}

TEST_CASE("binary parser: bad header returns BAD_FRAME", "[aj_sr04m][parser]") {
  uint8_t frame[4] = {0x00, 0x05, 0xDC, 0xE1};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
}

TEST_CASE("binary parser: short length returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  uint8_t frame[3] = {0xFF, 0x05, 0xDC};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_frame(frame, 3, &dist));
}

TEST_CASE("binary parser: long length returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  uint8_t frame[8] = {0xFF, 0x05, 0xDC, 0xE0, 0xFF, 0x05, 0xDC, 0xE0};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_frame(frame, 8, &dist));
}

TEST_CASE("binary parser: bad checksum returns BAD_CHECKSUM",
          "[aj_sr04m][parser]") {
  uint8_t frame[4] = {0xFF, 0x05, 0xDC, 0x00};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM,
                    aj_sr04m_parse_binary_frame(frame, 4, &dist));
}

/* === ASCII frame parser (mode 5) ======================================= */

TEST_CASE("ascii parser: valid frame at 1500 mm", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_ascii_frame("Gap=1500 mm\r\n", &dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("ascii parser: lower bound 200 mm", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_ascii_frame("Gap=200 mm\r\n", &dist));
  TEST_ASSERT_EQUAL_INT16(200, dist);
}

TEST_CASE("ascii parser: upper bound 4500 mm", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_ascii_frame("Gap=4500 mm\r\n", &dist));
  TEST_ASSERT_EQUAL_INT16(4500, dist);
}

TEST_CASE("ascii parser: leading garbage tolerated", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK, aj_sr04m_parse_ascii_frame(
                                          "garbage Gap=2500 mm\r\n", &dist));
  TEST_ASSERT_EQUAL_INT16(2500, dist);
}

TEST_CASE("ascii parser: below minimum returns NO_ECHO", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO,
                    aj_sr04m_parse_ascii_frame("Gap=199 mm\r\n", &dist));
}

TEST_CASE("ascii parser: 6016 mm returns NO_ECHO", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO,
                    aj_sr04m_parse_ascii_frame("Gap=6016 mm\r\n", &dist));
}

TEST_CASE("ascii parser: missing pattern returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_ascii_frame("Hello world\r\n", &dist));
}

TEST_CASE("ascii parser: malformed Gap line returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_ascii_frame("Gap=abc mm\r\n", &dist));
}

TEST_CASE("ascii parser: NULL input returns BAD_FRAME", "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_ascii_frame(NULL, &dist));
}

TEST_CASE("ascii parser: empty string returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_ascii_frame("", &dist));
}
