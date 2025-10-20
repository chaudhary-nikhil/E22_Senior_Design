# ESP32 CMakeLists.txt for ESP-IDF
cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name
project(FormSync)

# Add required components
set(EXTRA_COMPONENT_DIRS components)

# # ESP32 CMakeLists.txt for ESP-IDF
# cmake_minimum_required(VERSION 3.16)

# # If you have extra component dirs (like your custom components/), set them BEFORE project()
# # This can be omitted if you only use the default "components" under the project root,
# # but it's fine to keep it explicit.
# set(EXTRA_COMPONENT_DIRS
#     ${CMAKE_SOURCE_DIR}/components
# )

# # Pull in the ESP-IDF build system
# include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# # Define the project
# project(FormSync)
