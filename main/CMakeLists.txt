idf_component_register(
    SRCS "app_main_ble.c"
    INCLUDE_DIRS "."
    REQUIRES ble nvs_flash
)