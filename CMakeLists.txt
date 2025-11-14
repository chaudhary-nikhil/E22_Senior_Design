# ESP32 CMakeLists.txt for ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Set IDF_PATH if not already set
if(NOT DEFINED ENV{IDF_PATH})
    set(ENV{IDF_PATH} "/Users/Abathini/esp/esp-idf")
endif()

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name
project(FormSync)

# Add required components
set(EXTRA_COMPONENT_DIRS components)
