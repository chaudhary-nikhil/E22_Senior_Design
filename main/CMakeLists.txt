idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bus_i2c imu_bno055 wifi_server esp_driver_gpio esp_timer
)