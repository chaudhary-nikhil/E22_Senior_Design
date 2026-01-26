idf_component_register(SRCS "storage.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs sdmmc driver imu_bno055 esp_timer 
                                psram_buffer protobuf freertos heap)
