# ESP32 CMakeLists.txt for ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name
project(FormSync)

# Add required components
set(EXTRA_COMPONENT_DIRS components)
