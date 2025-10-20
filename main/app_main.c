
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"

// Your storage API
#include "storage.h"

// Match storage.c mount point
#define MOUNT_POINT   "/sdcard"

// ====== LED config ======
#define LED_GPIO        GPIO_NUM_4     // D4
#define LED_ACTIVE_LOW  0              // set 1 if your LED is active-low

static inline void led_write(int on) {
    int level = on ? 1 : 0;
    if (LED_ACTIVE_LOW) level = on ? 0 : 1;
    gpio_set_level(LED_GPIO, level);
}
static void led_init(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_write(0);
}
static void blink_n(uint32_t n, int on_ms, int off_ms) {
    vTaskDelay(pdMS_TO_TICKS(150));
    for (uint32_t i = 0; i < n; i++) {
        led_write(1); vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_write(0); vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}
static void error_halt(uint32_t code) {
    // repeat code every ~1s, but no rapid blinking otherwise
    for (;;) {
        blink_n(code, 180, 220);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
}

// ====== Helpers ======
static int count_lines_in_file(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;

    int lines = 0;
    int c;
    while ((c = fgetc(f)) != EOF) {
        if (c == '\n') lines++;
    }
    fclose(f);
    return lines;
}

// ====== main ======
void app_main(void) {
    led_init();

    // 0) Init storage (mount SD)
    if (storage_init() != ESP_OK || !storage_is_available()) {
        error_halt(1);  // storage init/mount failed
    }

    // 1) Start a recording session
    if (storage_start_session() != ESP_OK || !storage_is_recording()) {
        error_halt(2);  // couldn't start
    }

    // Grab filename now (will be cleared after stop)
    char fname[32];
    if (storage_get_current_filename(fname, sizeof(fname)) != ESP_OK || fname[0] == '\0') {
        // If we can't get the name, still try to proceed; but failing to get
        // the name means we won't be able to verify later. Treat as error.
        error_halt(2);
    }

    // 2) Write 10 rows
    uint32_t t0_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    for (int i = 0; i < 10; i++) {
        uint32_t t_ms = t0_ms + (uint32_t)(i * 10);
        float ax = 0.1f * i, ay = 0.2f * i, az = 9.81f;
        float gx = 0.01f * i, gy = 0.02f * i, gz = 0.0f;
        float tempC = 27.0f;

        if (storage_log_imu_sample(t_ms, ax, ay, az, gx, gy, gz, tempC) != ESP_OK) {
            error_halt(3); // write failure
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // 3) Stop session (flush/close)
    if (storage_stop_session() != ESP_OK) {
        error_halt(4);
    }

    // 4) Verify by counting lines in the file we just created
    // full path = MOUNT_POINT + "/" + fname
    char fullpath[64];
    // Safe join (bounds checked)
    int n = snprintf(fullpath, sizeof(fullpath), "%s/%s", MOUNT_POINT, fname);
    if (n < 0 || n >= (int)sizeof(fullpath)) {
        error_halt(5);
    }

    // Small delay to ensure VFS bookkeeping is visible
    vTaskDelay(pdMS_TO_TICKS(100));

    int lines = count_lines_in_file(fullpath);
    if (lines < 0) {
        error_halt(5); // couldn't open/read the just-created file
    }

    // Expect 1 header + 10 rows = 11 lines
    if (lines < 11) {
        error_halt(6); // not enough lines
    }

    // ----- LED report (blink only when needed) -----
    blink_n(3, 180, 220);                    // “new file created”
    blink_n((lines > 10) ? 10 : lines, 180, 220); // show up to 10 blinks for lines (caps at 10)
    led_write(1);                             // success: solid ON
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
