# SPDX-FileCopyrightText: 2026 Cyril Brunet
#
# SPDX-License-Identifier: MIT
#
# Linux-target test main. Pulls shared Unity sources from ../../cases/ so
# the QEMU and Linux runners exercise exactly the same tests.
set(CASES "${CMAKE_CURRENT_LIST_DIR}/../../cases")

idf_component_register(
    SRCS
        "${CASES}/main_test.c"
        "${CASES}/mocks.c"
        "${CASES}/test_parser.c"
        "${CASES}/test_mode1_2_gpio_rmt.c"
        "${CASES}/test_mode3_autonomous.c"
        "${CASES}/test_mode4_uart_binary.c"
        "${CASES}/test_mode5_uart_ascii.c"
    INCLUDE_DIRS "${CASES}"
    REQUIRES unity aj_sr04m esp_driver_uart
    WHOLE_ARCHIVE
)

# Wrap ESP-IDF UART functions so mode 3/4/5 tests can drive aj_sr04m
# without real hardware. On the linux target the wrap is enough on its
# own; the original esp_driver_uart symbols are absent because the linux
# port of the driver has no sources, and the linker rewrites the call
# sites in aj_sr04m.c to __wrap_uart_* before resolution.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=uart_driver_install"
    "-Wl,--wrap=uart_param_config"
    "-Wl,--wrap=uart_set_pin"
    "-Wl,--wrap=uart_write_bytes"
    "-Wl,--wrap=uart_read_bytes"
)

# GPIO + RMT + esp_rom wraps for modes 1/2. The linux port of
# esp_driver_gpio/esp_driver_rmt is empty (no sources, no real symbols),
# so unlike the QEMU build there is no IDF startup code calling these.
# The wraps are still gated on the mode to keep the linker quiet when
# the mocks aren't compiled.
if(CONFIG_AJ_SR04M_MODE_1 OR CONFIG_AJ_SR04M_MODE_2)
    target_link_libraries(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=gpio_config"
        "-Wl,--wrap=gpio_set_level"
        "-Wl,--wrap=esp_rom_delay_us"
        "-Wl,--wrap=rmt_new_rx_channel"
        "-Wl,--wrap=rmt_rx_register_event_callbacks"
        "-Wl,--wrap=rmt_enable"
        "-Wl,--wrap=rmt_receive"
    )
endif()
