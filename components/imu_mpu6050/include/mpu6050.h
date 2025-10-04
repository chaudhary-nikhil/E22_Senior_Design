#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR_GND 0x68
#define MPU6050_ADDR_VCC 0x69

typedef enum { MPU_AFS_2G=0, MPU_AFS_4G, MPU_AFS_8G, MPU_AFS_16G } mpu_accel_range_t;
typedef enum { MPU_GFS_250DPS=0, MPU_GFS_500DPS, MPU_GFS_1000DPS, MPU_GFS_2000DPS } mpu_gyro_range_t;

typedef struct {
    uint32_t t_ms;
    float ax, ay, az; // m/s^2
    float gx, gy, gz; // rad/s
    float tempC;
} imu_sample_t;

esp_err_t mpu6050_init(int port, uint8_t addr, mpu_accel_range_t a, mpu_gyro_range_t g);
esp_err_t mpu6050_read_sample(int port, uint8_t addr, imu_sample_t *out);

#ifdef __cplusplus
}
#endif
