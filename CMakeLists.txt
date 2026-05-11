# SPDX-FileCopyrightText: 2026 Cyril Brunet
#
# SPDX-License-Identifier: MIT

idf_component_register(SRCS "src/aj_sr04m.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES log esp_driver_gpio esp_driver_uart esp_driver_rmt)
