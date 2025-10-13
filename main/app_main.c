
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bus_i2c.h"
#include "mpu6050.h"
#include "serial_stream.h"
#include "storage.h"
#include "button.h"

static const char *TAG = "APP";

// Button configuration
#define BUTTON_GPIO 4  // Change this to your button GPIO pin
#define BUTTON_ACTIVE_HIGH false  // Set to true if button is active high

// Button callback function
static void button_callback(button_event_t event);

void app_main(void) {
    ESP_LOGI(TAG, "FormSync FW starting with button-controlled storage");

    // Init I2C (SDA=21, SCL=22 @ 400kHz)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 400000));

    // Init IMU with default ranges
    ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0, MPU6050_ADDR_GND, MPU_AFS_2G, MPU_GFS_250DPS));

    // Init storage system
    ESP_ERROR_CHECK(storage_init());

    // Init button
    button_config_t button_cfg = {
        .gpio_num = BUTTON_GPIO,
        .active_high = BUTTON_ACTIVE_HIGH,
        .debounce_ms = 50,
        .long_press_ms = 2000,
        .callback = button_callback
    };
    ESP_ERROR_CHECK(button_init(&button_cfg));

    // Init serial streaming
    ESP_ERROR_CHECK(serial_stream_init());

    ESP_LOGI(TAG, "All systems initialized!");
    ESP_LOGI(TAG, "Press button on GPIO %d to start/stop storage recording", BUTTON_GPIO);
    ESP_LOGI(TAG, "Starting real-time IMU streaming via serial");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FORMSYNC_SAMPLE_HZ);

    while (1) {
        imu_sample_t s;
        esp_err_t err = mpu6050_read_sample(I2C_NUM_0, MPU6050_ADDR_GND, &s);
        if (err == ESP_OK) {
            // Log to storage if recording
            if (storage_is_recording()) {
                storage_log_imu_sample(s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.tempC);
            }
            
            // Log IMU data
            ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s %s",
                        (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                        storage_is_recording() ? "[RECORDING]" : "[IDLE]");

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

// Button callback function
static void button_callback(button_event_t event) {
    switch (event) {
        case BUTTON_EVENT_PRESSED:
            if (storage_is_recording()) {
                // Stop recording
                ESP_ERROR_CHECK(storage_stop_session());
                ESP_LOGI(TAG, "*** STORAGE STOPPED ***");
            } else {
                // Start recording
                ESP_ERROR_CHECK(storage_start_session());
                ESP_LOGI(TAG, "*** STORAGE STARTED ***");
            }
            break;
            
        case BUTTON_EVENT_LONG_PRESSED:
            // Long press: delete all files
            ESP_LOGI(TAG, "*** LONG PRESS: DELETING ALL FILES ***");
            ESP_ERROR_CHECK(storage_delete_all_files());
            ESP_LOGI(TAG, "*** ALL FILES DELETED ***");
            break;
            
        case BUTTON_EVENT_RELEASED:
            // Short press release - nothing to do
            break;
            
        default:
            break;
    }
}
