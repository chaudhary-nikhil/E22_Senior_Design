idf_component_register(
    SRCS "haptic.c" "dtw_classifier.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_driver_ledc
)
