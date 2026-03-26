idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bus_i2c imu_bno055 storage wifi_server driver
            haptic stroke_detector nvs_flash protobuf led_strip esp_timer
)