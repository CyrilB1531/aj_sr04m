/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Parser unit tests — mode-agnostic.
 *
 * Both parsers are part of the public API (`aj_sr04m_parse_binary_frame`
 * for the 4-byte frame used by modes 3/4, `aj_sr04m_parse_ascii_frame`
 * for the `Gap=XXXX mm\r\n` payload of mode 5) and can be exercised
 * without any hardware mocks; tests here run in every build regardless
 * of the configured sensor mode.
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

/* === binary stream scan ================================================= */

/* 1500 mm = 0x05DC, checksum (0xFF + 0x05 + 0xDC) & 0xFF = 0xE0.
 * 2000 mm = 0x07D0, checksum 0xD6. */
#define FRAME_1500 0xFF, 0x05, 0xDC, 0xE0
#define FRAME_2000 0xFF, 0x07, 0xD0, 0xD6

TEST_CASE("binary stream: single frame matches the frame parser",
          "[aj_sr04m][parser]") {
  const uint8_t buf[4] = {FRAME_1500};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("binary stream: returns the newest of several frames",
          "[aj_sr04m][parser]") {
  const uint8_t buf[8] = {FRAME_1500, FRAME_2000};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
  TEST_ASSERT_EQUAL_INT16(2000, dist);
}

/* What an autonomous module actually leaves in the ring buffer: a read
 * starts mid-frame and ends mid-frame. This is the case that made every
 * mode 3 read fail before the scan existed. */
TEST_CASE("binary stream: skips leading and trailing partial frames",
          "[aj_sr04m][parser]") {
  const uint8_t buf[13] = {0xDC,       0xE0, /* tail of an earlier frame */
                           FRAME_1500,       /* complete */
                           FRAME_2000,       /* complete, newest */
                           0xFF,       0x07,
                           0xD0}; /* truncated, no checksum yet */
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
  TEST_ASSERT_EQUAL_INT16(2000, dist);
}

/* 0xFF is a legal dist_H: 65244 mm would be absurd, but the byte still
 * appears mid-frame and must not be mistaken for a header. The checksum is
 * what rules it out. */
TEST_CASE("binary stream: a 0xFF inside a frame is not taken for a header",
          "[aj_sr04m][parser]") {
  const uint8_t buf[8] = {0xFF, 0xFF, 0x01, 0xFF, FRAME_1500};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_OK,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
  TEST_ASSERT_EQUAL_INT16(1500, dist);
}

TEST_CASE("binary stream: all checksums wrong returns BAD_CHECKSUM",
          "[aj_sr04m][parser]") {
  const uint8_t buf[8] = {0xFF, 0x05, 0xDC, 0x00, 0xFF, 0x07, 0xD0, 0x00};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_CHECKSUM,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
}

TEST_CASE("binary stream: no header at all returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  const uint8_t buf[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
}

TEST_CASE("binary stream: out-of-range distance returns NO_ECHO",
          "[aj_sr04m][parser]") {
  /* 6016 mm = 0x1780, checksum (0xFF + 0x17 + 0x80) & 0xFF = 0x96 — the
   * out-of-range sentinel this AJ-SR04M revision emits. */
  const uint8_t buf[4] = {0xFF, 0x17, 0x80, 0x96};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_NO_ECHO,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
}

TEST_CASE("binary stream: buffer shorter than a frame returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  const uint8_t buf[3] = {0xFF, 0x05, 0xDC};
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_stream(buf, sizeof(buf), &dist));
}

TEST_CASE("binary stream: NULL buffer returns BAD_FRAME",
          "[aj_sr04m][parser]") {
  int16_t dist = 0;
  TEST_ASSERT_EQUAL(AJ_SR04M_DIST_BAD_FRAME,
                    aj_sr04m_parse_binary_stream(NULL, 8, &dist));
}
