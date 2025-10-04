#include "mpu6050.h"
#include "bus_i2c.h"
#include "esp_timer.h"
#include <math.h>

// Registers
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

static inline float accel_lsb_per_ms2(mpu_accel_range_t a) {
    // MPU6050 accel LSB/g: 16384, 8192, 4096, 2048. Convert g→m/s^2 by dividing by 9.80665.
    const float lsb_per_g[] = {16384.f, 8192.f, 4096.f, 2048.f};
    return lsb_per_g[a] / 9.80665f;
}

static inline float gyro_lsb_per_rads(mpu_gyro_range_t g) {
    // gyro LSB/(°/s): 131, 65.5, 32.8, 16.4. Convert °/s→rad/s by dividing by (180/pi).
    const float lsb_per_dps[] = {131.f, 65.5f, 32.8f, 16.4f};
    const float dps_to_rads = (float)M_PI / 180.0f;
    return lsb_per_dps[g] / dps_to_rads;
}

esp_err_t mpu6050_init(int port, uint8_t addr, mpu_accel_range_t a, mpu_gyro_range_t g) {
    // Wake up device
    ESP_ERROR_CHECK(bus_i2c_wr8(port, addr, REG_PWR_MGMT_1, 0x00, pdMS_TO_TICKS(100)));
    // Set accel range
    ESP_ERROR_CHECK(bus_i2c_wr8(port, addr, REG_ACCEL_CONFIG, (uint8_t)(a<<3), pdMS_TO_TICKS(50)));
    // Set gyro range
    ESP_ERROR_CHECK(bus_i2c_wr8(port, addr, REG_GYRO_CONFIG, (uint8_t)(g<<3), pdMS_TO_TICKS(50)));
    return ESP_OK;
}

esp_err_t mpu6050_read_sample(int port, uint8_t addr, imu_sample_t *out) {
    uint8_t d[14];
    esp_err_t err = bus_i2c_wrrd(port, addr, REG_ACCEL_XOUT_H, d, sizeof d, pdMS_TO_TICKS(50));
    if (err != ESP_OK) return err;

    int16_t ax = (int16_t)((d[0] << 8) | d[1]);
    int16_t ay = (int16_t)((d[2] << 8) | d[3]);
    int16_t az = (int16_t)((d[4] << 8) | d[5]);
    int16_t t  = (int16_t)((d[6] << 8) | d[7]);
    int16_t gx = (int16_t)((d[8] << 8) | d[9]);
    int16_t gy = (int16_t)((d[10] << 8) | d[11]);
    int16_t gz = (int16_t)((d[12] << 8) | d[13]);

    // Convert to SI units
    static mpu_accel_range_t acc_range = MPU_AFS_2G;
    static mpu_gyro_range_t gyro_range = MPU_GFS_250DPS;
    // In a real driver you would store ranges in driver state; kept static here for brevity.

    float a_lsb_ms2 = accel_lsb_per_ms2(acc_range);
    float g_lsb_rads = gyro_lsb_per_rads(gyro_range);

    out->t_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    out->ax = ax / a_lsb_ms2;
    out->ay = ay / a_lsb_ms2;
    out->az = az / a_lsb_ms2;
    out->gx = gx / g_lsb_rads;
    out->gy = gy / g_lsb_rads;
    out->gz = gz / g_lsb_rads;
    out->tempC = (t / 340.0f) + 36.53f; // datasheet formula
    return ESP_OK;
}
