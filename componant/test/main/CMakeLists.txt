idf_component_register(SRCS "main_test.c"
  INCLUDE_DIRS ""
  REQUIRES unity aj_sr04m
  WHOLE_ARCHIVE)

# gcov.ld defines __gcov_info_start/end so main_test.c can walk the gcov_info
# table when --coverage is on. Harmless when coverage is disabled (the section
# stays empty).
target_linker_script(${COMPONENT_LIB} INTERFACE "gcov.ld")

# Wrap ESP-IDF UART functions so mode 4 tests can drive aj_sr04m without real
# hardware. Mock implementations are in componant/aj_sr04m/test/mocks.c.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=uart_driver_install"
    "-Wl,--wrap=uart_param_config"
    "-Wl,--wrap=uart_set_pin"
    "-Wl,--wrap=uart_write_bytes"
    "-Wl,--wrap=uart_read_bytes"
)
