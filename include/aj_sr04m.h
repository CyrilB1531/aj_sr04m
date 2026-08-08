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
 * @brief Maximum number of sensor instances managed by the driver.
 *
 * Configured via `idf.py menuconfig` → AJ-SR04M Configuration. Bounds the
 * size of the arrays passed to aj_sr04m_read_all().
 */
#define AJ_SR04M_MAX_SENSORS CONFIG_AJ_SR04M_MAX_SENSORS

/**
 * @brief Delay inserted between consecutive triggers in aj_sr04m_trigger_all().
 *
 * Configured via `idf.py menuconfig` → AJ-SR04M Configuration. Milliseconds.
 * Zero fires every sensor back to back.
 *
 * @attention Co-located modules pick up each other's 40 kHz burst, which
 * competes with the real echo and makes the affected sensor report
 * AJ_SR04M_DIST_NO_ECHO on part of its readings. Keep a non-zero value
 * unless the modules are acoustically isolated from one another.
 */
#ifdef CONFIG_AJ_SR04M_TRIGGER_STAGGER_MS
#define AJ_SR04M_TRIGGER_STAGGER_MS CONFIG_AJ_SR04M_TRIGGER_STAGGER_MS
#else
/* Kconfig only exposes the option when several sensors are configured. */
#define AJ_SR04M_TRIGGER_STAGGER_MS 0
#endif

/**
 * @brief Opaque handle to an AJ-SR04M sensor instance.
 *
 * Created by aj_sr04m_new() and destroyed by aj_sr04m_delete().
 * Multiple instances can coexist on the same board (limited by available
 * RMT channels for GPIO modes, UART ports for UART modes).
 */
typedef struct aj_sr04m_sensor *aj_sr04m_handle_t;

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
 * @brief Initialize the AJ-SR04M driver.
 *
 * This function must be called once before creating any sensor instances.
 * It initializes the global driver state (GPIO, RMT, or UART hardware as
 * needed).
 *
 * @return
 *    - ESP_OK on success
 *    - ESP_ERR_NO_MEM if internal allocations fail
 *    - the error code returned by the hardware driver on failure
 */
esp_err_t aj_sr04m_init(void);

/**
 * @brief Release all sensor instances and reset the driver.
 *
 * Deletes every sensor created by aj_sr04m_init()/aj_sr04m_new() and returns
 * the driver to its pre-initialized state, so aj_sr04m_init() can run a fresh
 * configuration. Safe to call when the driver is not initialized.
 *
 * @return
 *    - ESP_OK on success
 */
esp_err_t aj_sr04m_deinit(void);

/**
 * @brief Create a new AJ-SR04M sensor instance.
 *
 * @param trigger_pin GPIO pin for TRIGGER (modes 1-2) or UART TX (modes 3-5)
 * @param echo_pin    GPIO pin for ECHO (modes 1-2) or UART RX (modes 3-5)
 * @param trigger_byte UART trigger byte (modes 4-5 only, ignored for modes
 * 1-3)
 * @param uart_num    UART port (modes 3-5). A value in [0, SOC_UART_NUM) uses
 * that hardware controller; any other value selects the software (RMT) UART
 * backend. Ignored in modes 1-2.
 *
 * @return
 *    - handle (non-NULL) on success
 *    - NULL if allocation fails or if hardware resources are exhausted
 *
 * @note
 *    - Modes 1-2 (GPIO): each instance requires a dedicated RMT RX channel.
 *      ESP32 typically provides 4-8 RMT channels; adjust
 * CONFIG_AJ_SR04M_MAX_SENSORS accordingly.
 *    - Modes 3-5 (UART): a hardware-backend instance needs a dedicated UART
 *      port (ESP32 provides 3, ports 0-2); a software-backend instance needs
 *      one RMT RX channel instead, lifting the per-port limit.
 *    - The instance operates in the global mode AJ_SR04M_MODE (configured via
 * menuconfig).
 */
aj_sr04m_handle_t aj_sr04m_new(int trigger_pin, int echo_pin,
                               uint8_t trigger_byte, int uart_num);

/**
 * @brief Delete (destroy) an AJ-SR04M sensor instance.
 *
 * Releases all resources associated with the sensor (GPIO, RMT channel, UART
 * driver, semaphores, etc.).
 *
 * @param handle Handle returned by aj_sr04m_new()
 */
void aj_sr04m_delete(aj_sr04m_handle_t handle);

/**
 * @brief Trigger a distance measurement on a specific sensor instance.
 *
 * Behavior depending on AJ_SR04M_MODE:
 *    - modes 1 and 2: arm RMT RX, then pulse the TRIGGER pin
 *    - modes 4 and 5: send the trigger byte over the UART
 *    - mode 3: no-op (the module emits frames autonomously)
 *
 * @param handle Handle returned by aj_sr04m_new()
 */
void aj_sr04m_trigger(aj_sr04m_handle_t handle);

/**
 * @brief Read the distance measured by a specific sensor instance.
 *
 * @param handle   Handle returned by aj_sr04m_new()
 * @param[out] distance distance in millimeters (valid only if the return
 * value is AJ_SR04M_DIST_OK)
 *
 * @return
 *    - AJ_SR04M_DIST_OK if the measurement is valid
 *    - AJ_SR04M_DIST_NO_ECHO if no echo was detected
 *    - AJ_SR04M_DIST_BAD_CHECKSUM if the UART checksum is invalid
 *    - AJ_SR04M_DIST_BAD_FRAME if the UART frame is malformed
 */
aj_sr04m_dist_status_t aj_sr04m_read_distance(aj_sr04m_handle_t handle,
                                              int16_t *distance);

/**
 * @brief Get the number of configured sensor instances.
 *
 * Sensors are configured from Kconfig when aj_sr04m_init() is called.
 *
 * @return number of configured sensors
 */
int aj_sr04m_get_sensor_count(void);

/**
 * @brief Trigger a distance measurement on all configured sensors.
 *
 * @return
 *    - ESP_OK if at least one sensor was triggered successfully
 *    - ESP_ERR_INVALID_STATE if the driver is not initialized or no sensors are
 * configured
 */
esp_err_t aj_sr04m_trigger_all(void);

/**
 * @brief Read distances from all configured sensors.
 *
 * @param[out] distances      array to receive measured distances in millimeters
 * @param[out] statuses       array to receive measurement statuses
 * @param[in]  max_sensors    capacity of the provided arrays
 * @param[out] out_sensor_count number of sensors read
 *
 * @return
 *    - ESP_OK on success
 *    - ESP_ERR_INVALID_ARG if arrays are NULL or max_sensors is too small
 *    - ESP_ERR_INVALID_STATE if the driver is not initialized or no sensors are
 * configured
 */
esp_err_t aj_sr04m_read_all(int16_t *distances,
                            aj_sr04m_dist_status_t *statuses, int max_sensors,
                            int *out_sensor_count);

#ifdef __cplusplus
}
#endif
