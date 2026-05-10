# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Paths to components that will be built in the test_app.
# If the components have unit tests (in its test subdirectory) it will be run too.
set(EXTRA_COMPONENT_DIRS "../aj_sr04m")

set(TEST_COMPONENTS "aj_sr04m" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
option(DISABLE_COVERAGE "Skip --coverage instrumentation on aj_sr04m" OFF)
project(ultrason_test)

# Coverage instrumentation: gcov emits .gcno files at build time and adds .bss
# counters that the runtime updates; the CI reconstructs .gcda from a UART dump
# produced by main_test.c. Scoped to aj_sr04m + its test component only —
# instrumenting the whole IDF blows past the ESP32 dram0_0_seg by ~140 KB.
# Pass -DDISABLE_COVERAGE=ON to opt out (e.g. for size-sensitive builds).
if(NOT DISABLE_COVERAGE)
  foreach(target aj_sr04m aj_sr04m_test)
    idf_component_get_property(lib ${target} COMPONENT_LIB)
    target_compile_options(${lib} PRIVATE --coverage)
    target_link_libraries(${lib} INTERFACE --coverage)
  endforeach()
endif()
