idf_component_register(SRCS "storage.c" 
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs sdmmc driver esp_timer imu_bno055)