// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"

// #include "bus_i2c.h"
// #include "bno055.h"
// #include "wifi_server.h"

// #include "storage.h"     // <-- new
// #include "bno_types.h"   // <-- your bno055_sample_t

// static const char *TAG = "GOLDENFORM";

// // Button GPIO
// #define TOGGLE_BUTTON_GPIO  0
// #define STATUS_LED_GPIO     2

// // Logging state
// static volatile bool logging_active = false;
// static volatile bool button_pressed = false;
// static volatile uint32_t last_button_time = 0;
// #define BUTTON_DEBOUNCE_MS 300

// // Session tracking
// static uint32_t session_data_count = 0;
// static uint32_t session_start_time = 0;

// // Forward decl: SD->WiFi sync task
// static void sync_sd_to_wifi_task(void* arg);

// // WiFi server callbacks
// static void wifi_status_callback(wifi_server_state_t state) {
//     ESP_LOGI(TAG, "WiFi server state changed to: %d", state);
// }

// static void session_status_callback(session_state_t state) {
//     ESP_LOGI(TAG, "Session state changed to: %d", state);
// }

// // Button ISR
// static void IRAM_ATTR button_isr_handler(void* arg) {
//     uint32_t current_time = esp_timer_get_time() / 1000;
//     if (current_time - last_button_time > BUTTON_DEBOUNCE_MS) {
//         button_pressed = true;
//         last_button_time = current_time;
//     }
// }

// // Buttons + LED
// static esp_err_t init_buttons(void) {
//     gpio_config_t toggle_config = {
//         .pin_bit_mask = (1ULL << TOGGLE_BUTTON_GPIO),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_NEGEDGE
//     };
//     ESP_ERROR_CHECK(gpio_config(&toggle_config));

//     gpio_config_t led_config = {
//         .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };
//     ESP_ERROR_CHECK(gpio_config(&led_config));

//     ESP_ERROR_CHECK(gpio_install_isr_service(0));
//     ESP_ERROR_CHECK(gpio_isr_handler_add(TOGGLE_BUTTON_GPIO, button_isr_handler, (void*) TOGGLE_BUTTON_GPIO));

//     gpio_set_level(STATUS_LED_GPIO, 0);
//     ESP_LOGI(TAG, "Buttons ready: GPIO%d toggle, GPIO%d LED", TOGGLE_BUTTON_GPIO, STATUS_LED_GPIO);
//     return ESP_OK;
// }

// static void update_led_status(void) {
//     gpio_set_level(STATUS_LED_GPIO, logging_active ? 1 : 0);
// }

// // Toggle logging session
// static void toggle_logging_session(void) {
//     if (!logging_active) {
//         // START logging
//         logging_active = true;
//         session_data_count = 0;
//         session_start_time = esp_timer_get_time() / 1000;

//         ESP_LOGI(TAG, "🟢 LOGGING START");
//         ESP_LOGI(TAG, "💾 SD binary logging ON (struct-batched)");
//         ESP_LOGI(TAG, "📡 Wi-Fi live stream disabled; will sync after stop");
//         update_led_status();

//         // Start SD storage
//         esp_err_t sderr = storage_init();
//         if (sderr != ESP_OK) ESP_LOGW(TAG, "storage_init: %s", esp_err_to_name(sderr));
//         sderr = storage_start_session();
//         if (sderr != ESP_OK) ESP_LOGE(TAG, "storage_start_session failed: %s", esp_err_to_name(sderr));

//         // WiFi server is still useful for UI / later sync endpoints
//         esp_err_t wifi_err = wifi_server_start_logging();
//         if (wifi_err != ESP_OK) {
//             ESP_LOGW(TAG, "wifi_server_start_logging: %s", esp_err_to_name(wifi_err));
//         }
//     } else {
//         // STOP logging
//         logging_active = false;
//         uint32_t session_duration = (esp_timer_get_time() / 1000) - session_start_time;

//         // Close SD session (flush)
//         esp_err_t sderr = storage_stop_session();
//         if (sderr != ESP_OK) ESP_LOGW(TAG, "storage_stop_session: %s", esp_err_to_name(sderr));

//         // Stop Wi-Fi logging mode (we’ll now upload from SD)
//         esp_err_t wifi_err = wifi_server_stop_logging();
//         if (wifi_err != ESP_OK) ESP_LOGW(TAG, "wifi_server_stop_logging: %s", esp_err_to_name(wifi_err));

//         ESP_LOGI(TAG, "🔴 LOGGING STOP");
//         ESP_LOGI(TAG, "⏱️  Duration: %u ms", session_duration);
//         ESP_LOGI(TAG, "📈 Samples:  %u", session_data_count);
//         ESP_LOGI(TAG, "📤 Starting SD → Wi-Fi sync...");
//         update_led_status();

//         // Spawn sync task
//         xTaskCreatePinnedToCore(sync_sd_to_wifi_task, "sync_sd", 4096, NULL, 4, NULL, tskNO_AFFINITY);

//         session_data_count = 0;
//     }
// }

// // Helper: push one sample to existing Wi-Fi API
// static inline void push_sample_to_wifi(const bno055_sample_t* s) {
//     (void)wifi_server_add_data_point(
//         s->t_ms,
//         s->ax, s->ay, s->az,
//         s->gx, s->gy, s->gz,
//         s->qw, s->qx, s->qy, s->qz,
//         s->sys_cal, s->gyro_cal, s->accel_cal, s->mag_cal
//     );
// }

// // Post-session: read .BIN files, send to Wi-Fi
// static void sync_sd_to_wifi_task(void* arg) {
//     // List files
//     char files[64][32];
//     uint32_t count = 0;
//     if (storage_list_files(files, 64, &count) != ESP_OK || count == 0) {
//         ESP_LOGI(TAG, "No files to sync.");
//         vTaskDelete(NULL);
//         return;
//     }

//     // Replay each BIN (CSV would need a parser)
//     for (uint32_t i = 0; i < count; ++i) {
//         const char* name = files[i];
//         if (!(strstr(name, ".BIN") || strstr(name, ".bin"))) continue;

//         char path[160];
//         snprintf(path, sizeof(path), "/sdcard/%s", name);
//         FILE* f = fopen(path, "rb");
//         if (!f) { ESP_LOGW(TAG, "Open failed: %s", name); continue; }

//         bno055_sample_t buf[256];
//         size_t got, sent = 0;

//         while ((got = fread(buf, sizeof(bno055_sample_t), 256, f)) > 0) {
//             for (size_t k = 0; k < got; ++k) {
//                 push_sample_to_wifi(&buf[k]);
//                 sent++;
//                 if ((sent & 0xFF) == 0) vTaskDelay(pdMS_TO_TICKS(1)); // keep system responsive
//             }
//         }
//         fclose(f);
//         ESP_LOGI(TAG, "Synced %u samples from %s", (unsigned)sent, name);

//         // Optional: unlink(path);  // delete after successful sync
//     }

//     ESP_LOGI(TAG, "✅ SD → Wi-Fi sync complete");
//     vTaskDelete(NULL);
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "🏊 GoldenForm Production Firmware Starting");

//     // I2C & IMU
//     esp_err_t i2c_err = bus_i2c_init(I2C_NUM_0, 21, 22, 100000);
//     if (i2c_err != ESP_OK) ESP_LOGE(TAG, "I2C init: %s", esp_err_to_name(i2c_err));
//     esp_err_t bno_err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
//     if (bno_err != ESP_OK) ESP_LOGE(TAG, "BNO055 init: %s", esp_err_to_name(bno_err));

//     // Buttons/LED
//     ESP_ERROR_CHECK(init_buttons());

//     // Wi-Fi server (still used for status + sync upload target)
//     esp_err_t wifi_init_status = wifi_server_init(wifi_status_callback, session_status_callback);
//     if (wifi_init_status != ESP_OK) {
//         ESP_LOGE(TAG, "WiFi init: %s", esp_err_to_name(wifi_init_status));
//     }

//     ESP_LOGI(TAG, "Press BOOT (GPIO%d) to toggle logging", TOGGLE_BUTTON_GPIO);

//     TickType_t t0 = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

//     while (1) {
//         if (button_pressed) { button_pressed = false; toggle_logging_session(); }

//         if (logging_active && bno_err == ESP_OK) {
//             bno055_sample_t s;
//             if (bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s) == ESP_OK) {
//                 // Write to SD (batched internally by storage.c)
//                 if (storage_enqueue_bno_sample(&s) == ESP_OK) {
//                     session_data_count++;
//                     if ((session_data_count % 50) == 0) {
//                         ESP_LOGI(TAG, "📊 Collected %u samples", session_data_count);
//                     }
//                 }
//             }
//         }

//         // Status every 5s
//         static int status_count = 0;
//         if ((status_count++ % 250) == 0) {
//             uint32_t file_samples = 0;
//             (void)storage_get_current_sample_count(&file_samples);
//             ESP_LOGI(TAG, "Status: Logging=%s, Storage=%d, FileSamples=%u",
//                      logging_active ? "ON" : "OFF",
//                      storage_get_state(),
//                      file_samples);
//         }

//         vTaskDelayUntil(&t0, period);
//     }
// }

// main/app_fake_generate.c
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "storage.h"
#include "bno055.h"

static const char* TAG = "FAKE_GEN";

static void fill_fake(bno055_sample_t* s, uint32_t t_ms, float k) {
    s->t_ms = t_ms;
    s->ax = 0.1f*sinf(k); s->ay = 0.1f*cosf(0.7f*k); s->az = 9.81f + 0.05f*sinf(1.3f*k);
    s->gx = 5.0f*sinf(0.5f*k); s->gy = 3.0f*cosf(0.9f*k); s->gz = 2.0f*sinf(1.7f*k);
    s->mx = 25; s->my = -5; s->mz = 45;
    s->roll=10*sinf(0.2f*k); s->pitch=8*cosf(0.3f*k); s->yaw=fmodf(0.5f*k*57.2958f, 360.0f);
    s->qw=1; s->qx=0; s->qy=0; s->qz=0;
    s->temp = 27.0f + 0.2f*sinf(0.1f*k);
    s->sys_cal = s->gyro_cal = s->accel_cal = s->mag_cal = 3;
}

void app_main(void) {
    ESP_LOGI(TAG, "Fake generate start (50Hz, 5 minutes, 1 file/min)");

    ESP_ERROR_CHECK(storage_init());
    ESP_ERROR_CHECK(storage_start_session());

    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz
    TickType_t last = xTaskGetTickCount();

    const uint32_t duration_ms = 5 * 60 * 1000; // 5 minutes
    uint32_t t0 = (uint32_t)(esp_timer_get_time()/1000ULL);
    uint32_t samples = 0;

    while (1) {
        vTaskDelayUntil(&last, period);
        uint32_t now_ms = (uint32_t)(esp_timer_get_time()/1000ULL);
        if (now_ms - t0 >= duration_ms) break;

        bno055_sample_t s;
        float k = 0.02f * (float)(now_ms - t0);
        fill_fake(&s, now_ms, k);

        if (storage_enqueue_bno_sample(&s) == ESP_OK) {
            samples++;
            if ((samples % 500) == 0) ESP_LOGI(TAG, "Samples: %u", (unsigned)samples);
        }
    }

    ESP_ERROR_CHECK(storage_stop_session());
    ESP_LOGI(TAG, "Done. Total samples=%u", (unsigned)samples);
    // Expect ~5 BIN files on /sdcard.
}


// #include <stdlib.h>
// #include "storage.h"
// #include "wifi_server.h"    // esp_err_t wifi_server_add_data_point(const bno055_sample_t*);

// // choose a capacity big enough; you can also do it in chunks (call multiple times)
// #define STACK_CAP  (200000)   // ~200k samples; adjust to your SD size & RAM

// void replay_all_sd_over_wifi(void)
// {
//     // 0) make sure SD is mounted: storage_init() called earlier in your boot
//     //    Wi-Fi server running: wifi_server_init(...)

//     bno055_sample_t *stack = (bno055_sample_t *)malloc(STACK_CAP * sizeof(bno055_sample_t));
//     if (!stack) {
//         ESP_LOGE("REPLAY", "malloc failed");
//         return;
//     }

//     size_t got = 0;
//     esp_err_t e = storage_read_all_bin_into_buffer(stack, STACK_CAP, &got);

//     ESP_LOGI("REPLAY", "Loaded %u samples from SD (ret=%s). Streaming to Wi-Fi...",
//              (unsigned)got, esp_err_to_name(e));

//     for (size_t i = 0; i < got; ++i) {
//         (void)wifi_server_add_data_point(&stack[i]);
//     }

//     ESP_LOGI("REPLAY", "Done sending %u samples.", (unsigned)got);
//     free(stack);

//     // If e == ESP_ERR_NO_MEM, you read a partial set. Call again (or use a bigger STACK_CAP)
//     // after you’ve consumed/sent these samples, or switch to a chunked approach (ask me and I’ll provide it).
// }
