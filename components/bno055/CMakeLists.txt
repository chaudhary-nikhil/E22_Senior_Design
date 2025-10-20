idf_component_register(
    SRCS "bno055.c"
    INCLUDE_DIRS "include"       # header lives in components/bno055/include
    REQUIRES driver esp_timer bus_i2c
)
