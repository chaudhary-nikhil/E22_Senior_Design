#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BNO055_OPMODE_CONFIG = 0x00,
    BNO055_OPMODE_NDOF  = 0x0C,
} bno055_opmode_t;

typedef struct {
    uint32_t t_ms;
    float ax, ay, az;   // m/s^2
    float gx, gy, gz;   // rad/s
    float tempC;
} bno_imu_sample_t;

esp_err_t bno055_init(i2c_port_t i2c, uint8_t addr);
esp_err_t bno055_config_units(bool accel_mps2, bool gyro_rads, bool temp_c);
esp_err_t bno055_set_opmode(bno055_opmode_t mode);
esp_err_t bno055_read_imu(bno_imu_sample_t* s);
