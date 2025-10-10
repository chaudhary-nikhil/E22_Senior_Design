#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "bus_i2c.h"
#include "mpu6050.h"
#include "ble_service.h"

static const char *TAG = "APP_BLE";

void app_main(void) {
    ESP_LOGI(TAG, "FormSync FW starting with BLE");

    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init I2C (SDA=21, SCL=22 @ 400kHz)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 400000));

    // Init IMU with default ranges
    ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0, MPU6050_ADDR_GND, MPU_AFS_2G, MPU_GFS_250DPS));

    // Init BLE service
    ESP_ERROR_CHECK(ble_init_if_enabled());

    ESP_LOGI(TAG, "All systems initialized, starting BLE IMU streaming");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 10Hz for BLE (lower rate to avoid congestion)

    while (1) {
        imu_sample_t s;
        esp_err_t err = mpu6050_read_sample(I2C_NUM_0, MPU6050_ADDR_GND, &s);
        if (err == ESP_OK) {
            // Log IMU data
            ESP_LOGI(TAG, "BLE: t=%u ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f",
                        (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

            // Send via BLE
            ble_send_imu_data(s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
        } else {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&t0, period);
    }
}

