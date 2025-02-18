# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Explicitly set ESP-IDF Components library's components path, so that the components are not interfered with the project's components
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components/esp_idf_lib_helpers
    ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components/i2cdev
    ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components/sht4x
    ${CMAKE_CURRENT_SOURCE_DIR}/esp-idf-lib/components/sgp40
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(espnow_example)
