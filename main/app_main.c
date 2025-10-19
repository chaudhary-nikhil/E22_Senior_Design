
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "serial_stream.h"

static const char *TAG = "APP";

void app_main(void) {
    ESP_LOGI(TAG, "FormSync FW starting");

    // Init I2C (SDA=21, SCL=22 @ 100kHz for BNO055 compatibility)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 100000));

    // Init BNO055 IMU (with improved timeout handling)
    ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, BNO055_ADDR_A));

    // Init serial streaming
    ESP_ERROR_CHECK(serial_stream_init());

    ESP_LOGI(TAG, "All systems initialized, starting real-time IMU streaming via serial");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz sampling (reduced for BNO055 stability)

    while (1) {
        bno055_sample_t s;
        esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s);
        if (err == ESP_OK) {
            // Log BNO055 data with calibration status
            ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s | mx=%.1f my=%.1f mz=%.1f μT | r=%.1f p=%.1f y=%.1f° | cal=%d%d%d%d",
                        (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, 
                        s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
                        s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal);

            // Create JSON with all 9-axis data and send via serial
            char json_data[512];
            snprintf(json_data, sizeof(json_data), 
                "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"temp\":%.1f,\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}}",
                (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
                s.qw, s.qx, s.qy, s.qz, s.temp,
                s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal);
            
            serial_stream_send_data(json_data);
        } else {
            ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&t0, period);
    }
}
