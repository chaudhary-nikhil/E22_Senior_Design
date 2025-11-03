
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c.h"

// #include "bus_i2c.h"
// //#include "mpu6050.h" using the 9 axis bno now 
// //#include "bno055.h"
// #include "serial_stream.h"
// #include "storage.h"
// #include "button.h"
//#include "driver/gpio.h"


// static const char *TAG = "APP";

// // Button configuration
// #define BUTTON_GPIO 4  // Change this to your button GPIO pin
// #define BUTTON_ACTIVE_HIGH false  // Set to true if button is active high

// // Button callback function
// static void button_callback(button_event_t event);

// void app_main(void) {
//     ESP_LOGI(TAG, "FormSync FW starting with button-controlled storage");

//     // Init I2C (SDA=21, SCL=22 @ 400kHz)
//     ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 400000));

//     // // Init IMU with default ranges
//     // //ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0, MPU6050_ADDR_GND, MPU_AFS_2G, MPU_GFS_250DPS));
//     // // Replace your mpu6050_init(...) with:
//     // ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, 0x28));  // ADR=GND → 0x28
//     // ESP_ERROR_CHECK(bno055_config_units(/* accel_mps2 = */ true,
//     //                                     /* gyro_rads = */ true,
//     //                                     /* temp_c = */ true));
//     // ESP_ERROR_CHECK(bno055_set_opmode(BNO055_OPMODE_NDOF)); // fusion on


//     // Init storage system
//     ESP_ERROR_CHECK(storage_init());

//     // Init button
//     button_config_t button_cfg = {
//         .gpio_num = BUTTON_GPIO,
//         .active_high = BUTTON_ACTIVE_HIGH,
//         .debounce_ms = 50,
//         .long_press_ms = 2000,
//         .callback = button_callback
//     };
//     ESP_ERROR_CHECK(button_init(&button_cfg));

//     // Init serial streaming
//     ESP_ERROR_CHECK(serial_stream_init());

//     ESP_LOGI(TAG, "All systems initialized!");
//     ESP_LOGI(TAG, "Press button on GPIO %d to start/stop storage recording", BUTTON_GPIO);
//     ESP_LOGI(TAG, "Starting real-time IMU streaming via serial");

//     TickType_t t0 = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FORMSYNC_SAMPLE_HZ);

//     while (1) {
//         // imu_sample_t s;
//         // esp_err_t err = mpu6050_read_sample(I2C_NUM_0, MPU6050_ADDR_GND, &s);
//         // if (err == ESP_OK) {
//         //     // Log to storage if recording
//         //     if (storage_is_recording()) {
//         //         storage_log_imu_sample(s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.tempC);
//         //     }
            
//         //     // Log IMU data
//         //     ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s %s",
//         //                 (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
//         //                 storage_is_recording() ? "[RECORDING]" : "[IDLE]");

//         //     // Create simple JSON and send via serial
//         //     char json_data[256];
//         //     snprintf(json_data, sizeof(json_data), 
//         //         "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
//         //         (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
            
//         //     serial_stream_send_data(json_data);
//         // } else {
//         //     ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
//         // }
//         // vTaskDelayUntil(&t0, period);
//         bno_imu_sample_t bs;
//         esp_err_t err = bno055_read_imu(&bs);  // fills accel (m/s^2), gyro (rad/s), tempC, t_ms
//         if (err == ESP_OK) {
//             if (storage_is_recording()) {
//                 storage_log_imu_sample(bs.t_ms, bs.ax, bs.ay, bs.az, bs.gx, bs.gy, bs.gz, bs.tempC);
//             }
//             ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f %s",
//                     (unsigned)bs.t_ms, bs.ax, bs.ay, bs.az, bs.gx, bs.gy, bs.gz,
//                     storage_is_recording() ? "[RECORDING]" : "[IDLE]");

//             char json_data[256];
//             snprintf(json_data, sizeof(json_data),
//                 "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
//                 (unsigned)bs.t_ms, bs.ax, bs.ay, bs.az, bs.gx, bs.gy, bs.gz);
//             serial_stream_send_data(json_data);
//         } else {
//             ESP_LOGW(TAG, "BNO read failed: %s", esp_err_to_name(err));
//         }
//         vTaskDelayUntil(&t0, period);

//     }
// }

// // Button callback function
// static void button_callback(button_event_t event) {
//     switch (event) {
//         case BUTTON_EVENT_PRESSED:
//             if (storage_is_recording()) {
//                 // Stop recording
//                 ESP_ERROR_CHECK(storage_stop_session());
//                 ESP_LOGI(TAG, "*** STORAGE STOPPED ***");
//             } else {
//                 // Start recording
//                 ESP_ERROR_CHECK(storage_start_session());
//                 ESP_LOGI(TAG, "*** STORAGE STARTED ***");
//             }
//             break;
            
//         case BUTTON_EVENT_LONG_PRESSED:
//             // Long press: delete all files
//             ESP_LOGI(TAG, "*** LONG PRESS: DELETING ALL FILES ***");
//             ESP_ERROR_CHECK(storage_delete_all_files());
//             ESP_LOGI(TAG, "*** ALL FILES DELETED ***");
//             break;
            
//         case BUTTON_EVENT_RELEASED:
//             // Short press release - nothing to do
//             break;
            
//         default:
//             break;
//     }
// }
// ///////////////////////////////////////////////////////////////////////////////////////
// #include <stdio.h>
// #include <math.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "driver/gpio.h"


// // KEEP these project headers
// #include "serial_stream.h"
// #include "storage.h"
// #include "button.h"
// // #include "bno055.h"  // <-- TEMPORARILY DISABLED

// static const char *TAG = "APP_SD_TEST";

// // Button config (same as before)
// #define BUTTON_GPIO 4
// #define BUTTON_ACTIVE_HIGH false

// static void button_callback(button_event_t event) {
//     switch (event) {
//         case BUTTON_EVENT_PRESSED:
//             if (storage_is_recording()) {
//                 ESP_ERROR_CHECK(storage_stop_session());
//                 ESP_LOGI(TAG, "*** STORAGE STOPPED ***");
//             } else {
//                 ESP_ERROR_CHECK(storage_start_session());
//                 ESP_LOGI(TAG, "*** STORAGE STARTED ***");
//             }
//             break;
//         case BUTTON_EVENT_LONG_PRESSED:
//             ESP_LOGI(TAG, "*** LONG PRESS: DELETING ALL FILES ***");
//             ESP_ERROR_CHECK(storage_delete_all_files());
//             ESP_LOGI(TAG, "*** ALL FILES DELETED ***");
//             break;
//         default:
//             break;
//     }
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "FormSync SD test (IMU disabled)");

//     // ---- DO NOT init BNO here (commented) ----
//     // ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, 0x28));
//     // ESP_ERROR_CHECK(bno055_config_units(true,true,true));
//     // ESP_ERROR_CHECK(bno055_set_opmode(BNO055_OPMODE_NDOF));

//     // Init storage system (SPI SD on CLK=18, MOSI=23, MISO=19, CS=5 per storage.c)
//     ESP_ERROR_CHECK(storage_init());
  
//     gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
//     gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);


//     // Init button (optional; handy to toggle logging)
//     button_config_t button_cfg = {
//         .gpio_num = BUTTON_GPIO,
//         .active_high = BUTTON_ACTIVE_HIGH,
//         .debounce_ms = 50,
//         .long_press_ms = 2000,
//         .callback = button_callback
//     };
//     ESP_ERROR_CHECK(button_init(&button_cfg));

//     // Init serial stream (optional; we’ll print JSON too)
//     ESP_ERROR_CHECK(serial_stream_init());

//     ESP_LOGI(TAG, "Press button on GPIO %d to start/stop storage recording", BUTTON_GPIO);

//     // OPTIONAL: auto-start a short session for sanity (uncomment to auto-create a CSV on boot)
//     // ESP_ERROR_CHECK(storage_start_session());

//     const uint32_t hz = CONFIG_FORMSYNC_SAMPLE_HZ;          // your existing rate
//     const TickType_t period = pdMS_TO_TICKS(1000 / hz);
//     TickType_t t0 = xTaskGetTickCount();

//     // Dummy signal generator state
//     float phase = 0.0f;
//     const float dphase = 2.0f * (float)M_PI * 1.0f / (float)hz; // 1 Hz sine over time

//     while (1) {
//         // Fake IMU sample fields (same order/units as storage expects)
//         uint32_t t_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
//         float ax = 9.81f * sinf(phase);
//         float ay = 9.81f * cosf(phase);
//         float az = 9.81f;            // pretend gravity on Z
//         float gx = 0.1f * sinf(phase);  // rad/s
//         float gy = 0.1f * cosf(phase);
//         float gz = 0.0f;
//         float tempC = 27.0f;

//         // Log to SD if recording
//         if (storage_is_recording()) {
//             storage_log_imu_sample(t_ms, ax, ay, az, gx, gy, gz, tempC);
//         }

//         // Console log (throttle to every 10th sample so it’s readable)
//         static uint32_t n = 0;
//         if ((n++ % 10) == 0) {
//             ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f %s",
//                      (unsigned)t_ms, ax, ay, az, gx, gy, gz,
//                      storage_is_recording() ? "[RECORDING]" : "[IDLE]");
//         }

//         // Send small JSON over serial (optional)
//        // Send small JSON over serial (throttled so UART/console doesn't block)
//         static uint32_t serial_tick = 0;
//         if ((serial_tick++ % 10) == 0) {
//             char json_data[256];
//             snprintf(json_data, sizeof(json_data),
//                     "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
//                     (unsigned)t_ms, ax, ay, az, gx, gy, gz);
//             serial_stream_send_data(json_data);
// }

//         // Advance dummy signal
//         phase += dphase;
//         if (phase > 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;

//         vTaskDelayUntil(&t0, period);
//     }
// }
// ///////////////////////////////////////////////////////////
// main/app_main.c
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "storage.h"    // your storage API

#define TAG "APP_SD_TEST"
#define MOUNT_POINT "/sdcard"

// 
// If FILE_ROLLOVER_MS in storage.c == 1000 (1s), then 12s gives ~12 files.
#define TEST_DURATION_MS   (12000)   // total time we keep logging
#define SAMPLE_PERIOD_MS   (50)      // write one sample every 50ms (~20Hz)

// ------- helpers to read back files from SD -------
static int count_lines_in_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int lines = 0, c;
    while ((c = fgetc(f)) != EOF) {
        if (c == '\n') lines++;
    }
    fclose(f);
    return lines;
}

static void dump_file_head(const char *path, int max_lines) {
    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGW(TAG, "  could not open for reading: %s", path);
        return;
    }
    ESP_LOGI(TAG, "  --- head(%d): %s ---", max_lines, path);
    char buf[160];
    int shown = 0;
    while (shown < max_lines && fgets(buf, sizeof(buf), f)) {
        // trim newline for pretty logging
        size_t L = strlen(buf);
        if (L && (buf[L-1] == '\n' || buf[L-1] == '\r')) buf[L-1] = '\0';
        ESP_LOGI(TAG, "    %s", buf);
        shown++;
    }
    if (!feof(f)) {
        ESP_LOGI(TAG, "    ...");
    }
    fclose(f);
}

// List all *.csv in /sdcard, print line counts and first few lines
static void list_and_show_csvs(void) {
    DIR *d = opendir(MOUNT_POINT);
    if (!d) {
        ESP_LOGE(TAG, "Failed to open %s to list files", MOUNT_POINT);
        return;
    }
    struct dirent *ent;
    int csv_count = 0;
    while ((ent = readdir(d)) != NULL) {
        const char *name = ent->d_name;
        // skip . and ..
        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) continue;

        // ends with .csv (case-insensitive)
        size_t len = strlen(name);
        if (len < 4) continue;
        const char *ext = name + (len - 4);
        if (strcasecmp(ext, ".csv") != 0) continue;

        char full[256];
        int n = snprintf(full, sizeof(full), "%s/%s", MOUNT_POINT, name);
        if (n < 0 || n >= (int)sizeof(full)) {
            ESP_LOGW(TAG, "Path too long, skipping: %s", name);
            continue;
        }

        int lines = count_lines_in_file(full);
        ESP_LOGI(TAG, "[%2d] %-24s  lines=%d", ++csv_count, name, lines);

        // show header + first 3 data lines (total 4 lines)
        dump_file_head(full, 4);
    }
    closedir(d);

    if (csv_count == 0) {
        ESP_LOGW(TAG, "No CSV files found at %s", MOUNT_POINT);
    } else {
        ESP_LOGI(TAG, "Total CSV files found: %d", csv_count);
    }
}

// ================== MAIN ==================
void app_main(void) {
    ESP_LOGI(TAG, "---- SD CSV rollover demo start ----");
    ESP_LOGI(TAG, "Tip: set FILE_ROLLOVER_MS in storage.c to 1000 for fast testing.");

    // 0) init storage
    esp_err_t err = storage_init();
    if (err != ESP_OK || !storage_is_available()) {
        ESP_LOGE(TAG, "storage_init() failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "storage_init OK");

    // 1) start session (wipes CSVs at start per your storage.c)
    err = storage_start_session();
    if (err != ESP_OK || !storage_is_recording()) {
        ESP_LOGE(TAG, "storage_start_session() failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "recording started");

    // 2) write garbage data for TEST_DURATION_MS
    uint64_t t_start_us = esp_timer_get_time();
    uint32_t t0_ms = (uint32_t)(t_start_us / 1000ULL);

    while (1) {
        uint64_t now_us = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now_us - t_start_us) / 1000ULL);
        if (elapsed_ms >= TEST_DURATION_MS) break;

        // fake IMU sample
        uint32_t t_ms = t0_ms + elapsed_ms;
        float ax = 0.001f * (float)elapsed_ms;
        float ay = 0.002f * (float)elapsed_ms;
        float az = 9.81f;
        float gx = 0.01f;
        float gy = 0.02f;
        float gz = 0.03f;
        float tempC = 27.5f;

        if (storage_log_imu_sample(t_ms, ax, ay, az, gx, gy, gz, tempC) != ESP_OK) {
            ESP_LOGE(TAG, "storage_log_imu_sample failed at %u ms", elapsed_ms);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }

    ESP_LOGI(TAG, "logging loop finished (~%u ms)", TEST_DURATION_MS);

    // 3) stop session (closes last file)
    err = storage_stop_session();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "storage_stop_session() failed: %s", esp_err_to_name(err));
        // continue anyway to see what’s on SD
    } else {
        ESP_LOGI(TAG, "recording stopped");
    }

    // 4) list CSVs and show their heads
    ESP_LOGI(TAG, "Listing CSVs and printing first lines:");
    list_and_show_csvs();

    ESP_LOGI(TAG, "---- SD CSV rollover demo DONE ----");
    // Idle forever
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}


