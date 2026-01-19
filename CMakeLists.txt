# ESP32 CMakeLists.txt for ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Set IDF_PATH if not already set
# First try workspace ESP-IDF, then check common installation locations
if(NOT DEFINED ENV{IDF_PATH})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/esp-idf/tools/cmake/project.cmake")
        set(ENV{IDF_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/esp-idf")
    elseif(EXISTS "$ENV{HOME}/esp/esp-idf/tools/cmake/project.cmake")
        set(ENV{IDF_PATH} "$ENV{HOME}/esp/esp-idf")
    endif()
endif()

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name
project(FormSync)

# Add required components
set(EXTRA_COMPONENT_DIRS components)
