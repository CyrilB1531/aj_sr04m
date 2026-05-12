# Build with ESP-IDF v5.4.3:
#   source ~/esp/v5.4.3/esp-idf/export.sh
#   idf.py build && idf.py qemu
# v6.0.1 enforces --orphan-handling=error and rejects gcov.ld; v5.4.3 tolerates
# the orphan placement so the link succeeds.
cmake_minimum_required(VERSION 3.16)

# The aj_sr04m component lives two levels up (repo root); ESP-IDF derives
# the component name from the directory name (`aj_sr04m`), matching the
# published-registry name.
set(EXTRA_COMPONENT_DIRS "../..")

set(TEST_COMPONENTS "aj_sr04m" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

project(aj_sr04m_test)
