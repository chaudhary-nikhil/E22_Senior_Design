idf_component_register(
    SRCS "haptic.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_driver_ledc
)
