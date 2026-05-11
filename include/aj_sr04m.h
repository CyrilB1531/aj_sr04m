/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Operating mode of the AJ-SR04M sensor.
 *
 * Configured via `idf.py menuconfig` → AJ-SR04M Configuration → Sensor
 * operating mode.
 *
 * Available values:
 *    - 1: GPIO trigger/echo, 10-15 us pulse
 *    - 2: GPIO trigger/echo, 1100 us pulse
 *    - 3: UART autonomous, binary frame emitted continuously (~100 ms)
 *    - 4: UART low-power, binary frame emitted after a UART trigger byte
 *    - 5: UART low-power, ASCII frame "Gap=XXXX mm" emitted after a UART
 * trigger byte
 */
#define AJ_SR04M_MODE CONFIG_AJ_SR04M_MODE

/**
 * @brief Status of a distance measurement.
 */
typedef enum {
  AJ_SR04M_DIST_OK,      /**< valid measurement */
  AJ_SR04M_DIST_NO_ECHO, /**< no echo: too close, too far, or absorbing material
                          */
  AJ_SR04M_DIST_BAD_CHECKSUM, /**< frame received but checksum is invalid */
  AJ_SR04M_DIST_BAD_FRAME, /**< malformed frame (incorrect header or length) */
} aj_sr04m_dist_status_t;

/**
 * @brief Parse a 4-byte binary frame from the AJ-SR04M (modes 3 and 4).
 *
 * Frame format: 0xFF | dist_H | dist_L | checksum, with checksum = (0xFF +
 * dist_H + dist_L) & 0xFF.
 *
 * @param[in]  data     pointer to the frame buffer
 * @param[in]  len      number of bytes in @p data (must be 4 for a valid frame)
 * @param[out] distance distance in millimeters (valid only if return is
 * AJ_SR04M_DIST_OK)
 *
 * @return
 *    - AJ_SR04M_DIST_OK if the frame is valid and the distance is in [200,
 * 4500] mm
 *    - AJ_SR04M_DIST_BAD_FRAME if @p len is not 4 or the header byte is not
 * 0xFF
 *    - AJ_SR04M_DIST_BAD_CHECKSUM if the checksum byte does not match
 *    - AJ_SR04M_DIST_NO_ECHO if the parsed distance is out of [200, 4500] mm
 */
aj_sr04m_dist_status_t aj_sr04m_parse_binary_frame(const uint8_t *data, int len,
                                                   int16_t *distance);

/**
 * @brief Parse an ASCII frame from the AJ-SR04M (mode 5).
 *
 * Frame format: "Gap=XXXX mm" possibly preceded by garbage bytes and followed
 * by CR/LF.
 *
 * @param[in]  data     NUL-terminated buffer containing the frame
 * @param[out] distance distance in millimeters (valid only if return is
 * AJ_SR04M_DIST_OK)
 *
 * @return
 *    - AJ_SR04M_DIST_OK if the "Gap=...mm" pattern is found and the distance is
 * in [200, 4500] mm
 *    - AJ_SR04M_DIST_BAD_FRAME if the pattern is missing or unparseable
 *    - AJ_SR04M_DIST_NO_ECHO if the parsed distance is out of [200, 4500] mm
 */
aj_sr04m_dist_status_t aj_sr04m_parse_ascii_frame(const char *data,
                                                  int16_t *distance);

/**
 * @brief Initialize the AJ-SR04M sensor.
 *
 * Configures the trigger GPIO + RMT RX capture (modes 1 and 2) or the UART
 * (modes 3, 4 and 5) according to AJ_SR04M_MODE.
 *
 * @return
 *    - ESP_OK on success
 *    - ESP_ERR_NO_MEM if internal allocations fail
 *    - the error code returned by the GPIO, RMT or UART driver on failure
 */
esp_err_t aj_sr04m_init(void);

/**
 * @brief Trigger a distance measurement.
 *
 * Behavior depending on AJ_SR04M_MODE:
 *    - modes 1 and 2: arm RMT RX, then pulse the TRIGGER pin
 *    - modes 4 and 5: send a 0x01 byte over the UART
 *    - mode 3: no-op (the module emits frames autonomously)
 */
void aj_sr04m_trigger(void);

/**
 * @brief Read the distance measured by the sensor.
 *
 * @param[out] distance distance in millimeters (valid only if the return value
 * is AJ_SR04M_DIST_OK)
 *
 * @return
 *    - AJ_SR04M_DIST_OK if the measurement is valid
 *    - AJ_SR04M_DIST_NO_ECHO if no echo was detected
 *    - AJ_SR04M_DIST_BAD_CHECKSUM if the UART checksum is invalid
 *    - AJ_SR04M_DIST_BAD_FRAME if the UART frame is malformed
 */
aj_sr04m_dist_status_t aj_sr04m_read_duration(int16_t *distance);

#ifdef __cplusplus
}
#endif
