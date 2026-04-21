idf_component_register(SRCS "storage.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs sdmmc driver esp_driver_gpio imu_bno055 esp_timer 
                                psram_buffer protobuf freertos heap)
