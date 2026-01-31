# PSRAM Ring Buffer Component for ESP-IDF

idf_component_register(
    SRCS 
        "psram_buffer.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES
        imu_bno055
        freertos
        esp_common
        heap
)
