# SPDX-FileCopyrightText: 2026 Cyril Brunet
#
# SPDX-License-Identifier: MIT
#
# Linux-target test project. Builds the same Unity test cases as the QEMU
# project at tests/, but on the POSIX/Linux simulator so gcov data lands
# directly in build/ and lcov/gcovr can read it without a UART dump dance.
#
# Build with ESP-IDF v5.4.3:
#   source ~/esp/v5.4.3/esp-idf/export.sh
#   idf.py --preview set-target linux
#   idf.py build
#   ./build/aj_sr04m_test_linux.elf
cmake_minimum_required(VERSION 3.16)

# The aj_sr04m component lives two levels up; ESP-IDF derives the component
# name from the directory name (`aj_sr04m`).
set(EXTRA_COMPONENT_DIRS "../..")

# Restrict to what the tests actually need. Without this the linux target
# pulls in nvs_flash/mbedtls/lwip/esp_http_*, and one of them (nvs_flash)
# hard-requires libbsd-dev that we cannot assume is installed on the host.
set(COMPONENTS main aj_sr04m unity esp_driver_uart esp_driver_gpio
    esp_driver_rmt esp_rom log esp_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

# Force-include a small shim that fills in the SOC_UART_* caps and the
# `soc_periph_uart_clk_src_legacy_t` typedef that ESP-IDF's linux target
# omits. Without this, `driver/uart.h` (required by aj_sr04m.c in mode 4
# and by mocks.h) fails to compile on linux. See shims/uart_caps_shim.h.
idf_build_set_property(COMPILE_OPTIONS
    "-include;${CMAKE_CURRENT_LIST_DIR}/shims/uart_caps_shim.h" APPEND)

# esp_driver_gpio/esp_driver_rmt are registered as empty components on
# linux (no INCLUDE_DIRS), so `driver/gpio.h` and `driver/rmt_rx.h` are
# unavailable from the IDF tree. Add the local stub headers under
# shims/include/ to the global include path so mode 1/2 sources compile
# on linux; mocks.c intercepts every GPIO/RMT call so no real driver is
# needed at runtime.
idf_build_set_property(COMPILE_OPTIONS
    "-I${CMAKE_CURRENT_LIST_DIR}/shims/include" APPEND)

project(aj_sr04m_test_linux)

# Force Coverage instrumentation on the linux target: gcov writes .gcda files
# directly to the build tree on process exit, so no UART dump is required.
foreach(target aj_sr04m main)
  idf_component_get_property(lib ${target} COMPONENT_LIB)
  target_compile_options(${lib} PRIVATE --coverage -fno-merge-constants)
  target_link_libraries(${lib} INTERFACE --coverage)
endforeach()
