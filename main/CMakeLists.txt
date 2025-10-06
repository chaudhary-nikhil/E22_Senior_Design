idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES bus_i2c imu_mpu6050 serial_stream
)