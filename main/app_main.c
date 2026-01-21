
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "serial_stream.h"
#include "storage.h"

#define DEBUG_SD_CARD 0
#define PRINT_INTERVAL 100

static const char *TAG = "APP";

void app_main(void) {
    ESP_LOGI(TAG, "FormSync FW starting");

    // Init I2C (SDA=21, SCL=22 @ 100kHz for BNO055 compatibility)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 100000));

    // Init BNO055 IMU (with graceful error handling)
    esp_err_t bno_err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
    if (bno_err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 initialization failed: %s", esp_err_to_name(bno_err));
        ESP_LOGE(TAG, "Please check BNO055 wiring: SDA=21, SCL=22, VCC=3.3V, GND=GND");
        ESP_LOGE(TAG, "Continuing without BNO055 - no data will be logged");
        // Don't crash, just continue without IMU
    } else {
        ESP_LOGI(TAG, "BNO055 initialized successfully");
    }

    // Init serial streaming
    ESP_ERROR_CHECK(serial_stream_init());

    // Init SD card storage (with graceful error handling)
    esp_err_t storage_err = storage_init();
    if (storage_err != ESP_OK) {
        ESP_LOGW(TAG, "SD card storage initialization failed: %s", esp_err_to_name(storage_err));
        ESP_LOGW(TAG, "Continuing without SD card - data will only be sent via serial");
    } else {
        ESP_LOGI(TAG, "SD card storage initialized successfully");
        // Start recording session immediately
        storage_err = storage_start_session();
        if (storage_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start storage session: %s", esp_err_to_name(storage_err));
        } else {
            ESP_LOGI(TAG, "SD card recording session started");
        }
    }

    ESP_LOGI(TAG, "All systems initialized, starting real-time IMU streaming via serial and SD card");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FORMSYNC_SAMPLE_HZ); // Configurable sampling rate
    int count = 0;
    while (1) {
        if (bno_err == ESP_OK) {
            count++;
            // BNO055 is available, read real data
            bno055_sample_t s;
            esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s);
            if (err == ESP_OK) {
                // Create JSON with all 9-axis data and send via serial
                char json_data[512];
                if(count == PRINT_INTERVAL) {
                    snprintf(json_data, sizeof(json_data),
                    "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"lia_x\":%.3f,\"lia_y\":%.3f,\"lia_z\":%.3f,\"temp\":%.1f,\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}}",
                    (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                    s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
                    s.qw, s.qx, s.qy, s.qz, s.lia_x, s.lia_y, s.lia_z, s.temp,
                    s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal);
                    count = 0;
                }

                serial_stream_send_data(json_data);

                // Also write to SD card if storage is available
                if (storage_err == ESP_OK && storage_is_recording()) {
                    // For debugging purposes, can you write 1's for all IMU data?
                    if(DEBUG_SD_CARD) {
                        s.ax = 1.0f;
                        s.ay = 1.0f;
                        s.az = 1.0f;
                        s.gx = 1.0f;
                        s.gy = 1.0f;
                        s.gz = 1.0f;
                        s.mx = 1.0f;
                        s.my = 1.0f;
                        s.mz = 1.0f;
                        s.roll = 1.0f;
                        s.pitch = 1.0f;
                        s.yaw = 1.0f;
                        s.qw = 1.0f;
                        s.qx = 1.0f;
                        s.qy = 1.0f;
                        s.qz = 1.0f;
                        s.lia_x = 1.0f;
                        s.lia_y = 1.0f;
                        s.lia_z = 1.0f;
                        s.temp = 1.0f;
                        s.sys_cal = 1;
                        s.gyro_cal = 1;
                        s.accel_cal = 1;
                        s.mag_cal = 1;
                    }
                    
                    esp_err_t sd_err = storage_enqueue_bno_sample(&s);
                    if (sd_err != ESP_OK && sd_err != ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "SD card write failed: %s", esp_err_to_name(sd_err));
                    }
                }
            } else {
                ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
            }
        } else {
            // BNO055 not available, send status message
            static int status_count = 0;
            if (status_count % 50 == 0) { // Every 1 second at 50Hz
                ESP_LOGW(TAG, "BNO055 not available - no data being logged");
            }
            status_count++;
        }
        vTaskDelayUntil(&t0, period);
    }
}
