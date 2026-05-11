# SPDX-FileCopyrightText: 2026 Cyril Brunet
#
# SPDX-License-Identifier: MIT
#
# QEMU runner main. Pulls shared Unity sources from ../../cases/ so the
# QEMU and Linux runners exercise exactly the same tests.
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
    REQUIRES unity aj_sr04m esp_driver_uart esp_driver_gpio esp_driver_rmt
            esp_rom
    WHOLE_ARCHIVE
)

# UART wraps — used by modes 3, 4, 5 and harmless when the rest of the
# build does not reference them.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=uart_driver_install"
    "-Wl,--wrap=uart_param_config"
    "-Wl,--wrap=uart_set_pin"
    "-Wl,--wrap=uart_write_bytes"
    "-Wl,--wrap=uart_read_bytes"
)

# GPIO + RMT + esp_rom wraps are only added for modes 1/2. These symbols
# are also called by ESP-IDF internals (cpu_start, peripheral init), so
# we must not wrap them on builds where our mocks aren't compiled —
# otherwise the IDF boot path tries to call an undefined __wrap_*.
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
