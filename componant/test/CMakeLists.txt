# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Paths to components that will be built in the test_app.
# If the components have unit tests (in its test subdirectory) it will be run too.
set(EXTRA_COMPONENT_DIRS "../")

set(TEST_COMPONENTS "aj_sr04m" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(ultrason_test)
