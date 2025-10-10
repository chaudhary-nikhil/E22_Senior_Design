
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bus_i2c.h"
#include "mpu6050.h"
#include "serial_stream.h"

static const char *TAG = "APP";

void app_main(void) {
    ESP_LOGI(TAG, "FormSync FW starting");

    // Init I2C (SDA=21, SCL=22 @ 400kHz)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 400000));

    // Init IMU with default ranges
    ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0, MPU6050_ADDR_GND, MPU_AFS_2G, MPU_GFS_250DPS));

    // Init serial streaming
    ESP_ERROR_CHECK(serial_stream_init());

    ESP_LOGI(TAG, "All systems initialized, starting real-time IMU streaming via serial");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100Hz sampling

    while (1) {
        imu_sample_t s;
        esp_err_t err = mpu6050_read_sample(I2C_NUM_0, MPU6050_ADDR_GND, &s);
        if (err == ESP_OK) {
            // Log IMU data
            ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s",
                        (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);

            // Create simple JSON and send via serial
            char json_data[256];
            snprintf(json_data, sizeof(json_data), 
                "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
                (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
            
            serial_stream_send_data(json_data);
        } else {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&t0, period);
    }
}
