idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bus_i2c esp_timer)
