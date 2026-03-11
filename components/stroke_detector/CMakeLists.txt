idf_component_register(
    SRCS "stroke_detector.c"
    INCLUDE_DIRS "include"
    REQUIRES imu_bno055 haptic driver
)
