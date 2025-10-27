idf_component_register(SRCS "storage.c" "storage_old.c"
                       INCLUDE_DIRS "include"
                       REQUIRES fatfs sdmmc driver esp_timer)
