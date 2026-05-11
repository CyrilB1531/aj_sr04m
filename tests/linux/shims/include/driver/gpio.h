/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Stub `driver/gpio.h` for the ESP-IDF linux target test build.
 *
 * The real esp_driver_gpio component is empty on linux (no INCLUDE_DIRS,
 * no sources — see components/esp_driver_gpio/CMakeLists.txt), so this
 * header is missing from the build. The hal component still exposes
 * `hal/gpio_types.h` + `soc/gpio_num.h` on linux, so we reuse those for
 * enums and `gpio_num_t`, and add only what's missing: the gpio_config_t
 * aggregate and the two function prototypes the mocks intercept.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint64_t pin_bit_mask;
  gpio_mode_t mode;
  gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en;
  gpio_int_type_t intr_type;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *pGPIOConfig);
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);

#ifdef __cplusplus
}
#endif
