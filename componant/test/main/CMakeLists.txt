idf_component_register(SRCS "main_test.c"
  INCLUDE_DIRS ""
  REQUIRES unity aj_sr04m
  WHOLE_ARCHIVE)

# Wrap ESP-IDF UART functions so mode 4 tests can drive aj_sr04m without real
# hardware. Mock implementations are in componant/aj_sr04m/test/mocks.c.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--wrap=uart_driver_install"
    "-Wl,--wrap=uart_param_config"
    "-Wl,--wrap=uart_set_pin"
    "-Wl,--wrap=uart_write_bytes"
    "-Wl,--wrap=uart_read_bytes"
)
