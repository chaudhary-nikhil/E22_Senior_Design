/**
 * @file app_main.c
 * @brief GoldenForm Firmware - ESP32-S3-WROOM-1
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "bus_i2c.h"
#include "bno055.h"
#include "storage.h"
#include "wifi_server.h"

#define DEBUG_SD_CARD 0

static const char *TAG = "GOLDENFORM";

// ============================================================================
// Pin Definitions (from Kconfig, matching schematic)
// ============================================================================

// BNO055 I2C
#define I2C_SDA_GPIO        CONFIG_FORMSYNC_I2C_SDA_GPIO        // GPIO 4
#define I2C_SCL_GPIO        CONFIG_FORMSYNC_I2C_SCL_GPIO        // GPIO 5
#define BNO055_ADDR_GPIO    CONFIG_FORMSYNC_BNO055_ADDR_GPIO    // GPIO 6

// Button: Using BOOT button (GPIO0) on ESP32-S3 module
#define BUTTON_GPIO         0

// LED Configuration
// Set LED_ENABLED to 0 to disable LED (required when no LED is wired or GPIO conflicts)
// GPIO48 is used for SD card CS, so LED is disabled to avoid conflict
#define LED_ENABLED         0
#define LED_GPIO            48  // Not used when LED_ENABLED=0

// Status LEDs on GPIO 40, 41, 42
#define POWER_LED_GPIO      40  // Always on when powered
#define STATUS_LED_GPIO     41  // ON during logging, BLINKS during syncing
#define ERROR_LED_GPIO      42  // ON when any error occurs (IMU, SD card, WiFi)

// ============== Application State Machine ==============
typedef enum {
    STATE_IDLE,         // Not doing anything - press to record, hold to sync
    STATE_LOGGING,      // Recording IMU data to SD card
    STATE_SYNCING       // WiFi active, serving data
} app_state_t;

static volatile app_state_t current_state = STATE_IDLE;
static volatile uint32_t last_press_time = 0;
static volatile uint32_t last_release_time = 0;
#define BUTTON_DEBOUNCE_MS 80

// Button hold detection
static volatile uint32_t button_press_start_ms = 0;
static volatile bool button_is_pressed = false;
static volatile bool button_short_press = false;
static volatile bool button_hold_triggered = false;
#define BUTTON_HOLD_MS 1500
#define BUTTON_SHORT_MAX_MS 800

// Session stats
static uint32_t session_sample_count = 0;
static uint32_t session_start_time = 0;

// Subsystem status
static bool bno055_available = false;
static bool storage_available = false;
static bool wifi_available = false;
static bool system_has_error = false;

// ============== Error LED Control ==============
static void error_led_set(bool on) {
    gpio_set_level(ERROR_LED_GPIO, on ? 1 : 0);
    if (on) {
        system_has_error = true;
        ESP_LOGW(TAG, "Error LED ON (GPIO%d)", ERROR_LED_GPIO);
    }
}

// LED variables (only used when LED_ENABLED=1)
#if LED_ENABLED
static TaskHandle_t led_blink_task_handle = NULL;
static volatile bool led_blink_stop = false;
static led_strip_handle_t led_strip = NULL;
#endif

// ============== Button ISR (both edges for press/hold detection) ==============
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t now = esp_timer_get_time() / 1000;
    int level = gpio_get_level(BUTTON_GPIO);

    if (level == 0) {
        // Pressed down - debounce against last press
        if (now - last_press_time < BUTTON_DEBOUNCE_MS) return;
        last_press_time = now;
        button_is_pressed = true;
        button_press_start_ms = now;
        button_hold_triggered = false;
    } else {
        // Released - debounce against last release only
        if (now - last_release_time < BUTTON_DEBOUNCE_MS) return;
        last_release_time = now;
        if (button_is_pressed && !button_hold_triggered) {
            uint32_t held = now - button_press_start_ms;
            if (held > 50 && held < BUTTON_SHORT_MAX_MS) {
                button_short_press = true;
            }
        }
        button_is_pressed = false;
    }
}

// ============== LED Control ==============
// LED control functions (only compiled when LED_ENABLED=1)
#if LED_ENABLED
static void led_set(bool on) {
    if (led_strip == NULL) {
        return;
    }
    
    if (on) {
        esp_err_t ret = led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        if (ret == ESP_OK) {
            led_strip_refresh(led_strip);
        }
        ESP_LOGI(TAG, "LED ON (GPIO%d - Red)", LED_GPIO);
    } else {
        led_strip_clear(led_strip);
        ESP_LOGI(TAG, "LED OFF (GPIO%d)", LED_GPIO);
    }
}
#else
// LED disabled - empty function
static void led_set(bool on) {
    (void)on;  // Suppress unused parameter warning
}
#endif

#if LED_ENABLED
static void led_blink_task(void *arg) {
    ESP_LOGI(TAG, "LED blink task started");
    led_blink_stop = false;
    
    if (led_strip == NULL) {
        led_blink_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    
    while (!led_blink_stop) {
        led_strip_set_pixel(led_strip, 0, 255, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(250));
        if (led_blink_stop) break;
        
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    led_strip_clear(led_strip);
    led_blink_task_handle = NULL;
    vTaskDelete(NULL);
}

static void led_blink_start(void) {
    if (led_blink_task_handle == NULL) {
        led_blink_stop = false;
        xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, &led_blink_task_handle);
    }
}

static void led_blink_stop_and_wait(void) {
    if (led_blink_task_handle != NULL) {
        led_blink_stop = true;
        for (int i = 0; i < 20 && led_blink_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    if (led_strip != NULL) {
        led_strip_clear(led_strip);
    }
}
#else
// LED disabled - empty functions
static void led_blink_start(void) { }
static void led_blink_stop_and_wait(void) { }
#endif

// ============== Status LED (GPIO 41) Blink Control for Syncing ==============
static TaskHandle_t status_led_blink_task_handle = NULL;
static volatile bool status_led_blink_stop = false;

static void status_led_blink_task(void *arg) {
    ESP_LOGI(TAG, "Status LED blink task started (GPIO%d)", STATUS_LED_GPIO);
    status_led_blink_stop = false;
    
    while (!status_led_blink_stop) {
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        if (status_led_blink_stop) break;
        
        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    gpio_set_level(STATUS_LED_GPIO, 0);
    status_led_blink_task_handle = NULL;
    vTaskDelete(NULL);
}

static void status_led_blink_start(void) {
    if (status_led_blink_task_handle == NULL) {
        status_led_blink_stop = false;
        xTaskCreate(status_led_blink_task, "status_led_blink", 2048, NULL, 5, &status_led_blink_task_handle);
    }
}

static void status_led_blink_stop_and_wait(void) {
    if (status_led_blink_task_handle != NULL) {
        status_led_blink_stop = true;
        for (int i = 0; i < 20 && status_led_blink_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    gpio_set_level(STATUS_LED_GPIO, 0);
}

// ============== State Transitions ==============
static void transition_to_logging(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: LOGGING");
    ESP_LOGI(TAG, "Recording IMU data to SD card...");
    
    session_sample_count = 0;
    session_start_time = esp_timer_get_time() / 1000;
    
    led_blink_stop_and_wait();
    status_led_blink_stop_and_wait();
    
    if (storage_available) {
        esp_err_t err = storage_start_session();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start storage session: %s", esp_err_to_name(err));
            error_led_set(true);
        } else {
            ESP_LOGI(TAG, "Session #%" PRIu32 " started", storage_get_session_number());
        }
    }
    
    gpio_set_level(STATUS_LED_GPIO, 1);
    led_set(true);
    current_state = STATE_LOGGING;
    
    ESP_LOGI(TAG, "[LED] Status=SOLID Power=ON Error=OFF -> Recording active");
    ESP_LOGI(TAG, "Press to STOP, hold to SYNC.");
    ESP_LOGI(TAG, "========================================");
}

static void transition_to_syncing(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> SYNC requested");
    
    if (!wifi_available) {
        ESP_LOGW(TAG, "WiFi not available - cannot sync");
        ESP_LOGI(TAG, "========================================");
        return;
    }

    // If currently logging, stop the session first
    if (current_state == STATE_LOGGING && storage_available) {
        uint32_t dur = (uint32_t)(esp_timer_get_time() / 1000) - session_start_time;
        ESP_LOGI(TAG, "Auto-stopping session (%" PRIu32 " ms, %" PRIu32 " samples)",
                 dur, session_sample_count);
        storage_stop_session();
        gpio_set_level(STATUS_LED_GPIO, 0);
        led_set(false);
    }

    // Check if there's actually data to sync BEFORE entering sync state
    esp_err_t err = wifi_server_start_sync();
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "No sessions on SD card - record a session first");
        ESP_LOGI(TAG, "[LED] Quick blink -> no data, staying IDLE");
        // Brief error indication: blink status LED 3 times quickly
        for (int i = 0; i < 3; i++) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "========================================");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start sync: %s", esp_err_to_name(err));
        error_led_set(true);
        ESP_LOGI(TAG, "========================================");
        return;
    }

    // Data exists - now enter sync state
    current_state = STATE_SYNCING;
    status_led_blink_start();
    led_blink_start();
    ESP_LOGI(TAG, ">>> STATE: SYNCING");
    ESP_LOGI(TAG, "[LED] Status=BLINK Power=BLINK -> Wireless sync active");
    ESP_LOGI(TAG, "Connect to WiFi: %s (pw: %s)", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "Blink stops automatically after data transfer.");
    ESP_LOGI(TAG, "========================================");
}

static void transition_to_idle(bool after_sync) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: IDLE");
    
    // Stop recording if still active
    if (current_state == STATE_LOGGING && storage_available) {
        storage_stop_session();
    }
    
    led_blink_stop_and_wait();
    status_led_blink_stop_and_wait();
    
    if (current_state == STATE_SYNCING && wifi_available) {
        wifi_server_stop_sync();
    }
    
    // After a successful sync, clear the SD card for fresh sessions
    if (after_sync && storage_available) {
        ESP_LOGI(TAG, "Clearing synced data from SD card...");
        storage_delete_all_files();
        storage_reset_session_counter();
    }
    
    gpio_set_level(STATUS_LED_GPIO, 0);
    led_set(false);
    current_state = STATE_IDLE;
    session_sample_count = 0;
    
    ESP_LOGI(TAG, "[LED] Status=OFF Power=ON Error=OFF -> Idle");
    ESP_LOGI(TAG, "Press to start recording, hold to sync.");
    ESP_LOGI(TAG, "========================================");
}

// ============== Button Handlers ==============
static void handle_button_press(void) {
    switch (current_state) {
        case STATE_IDLE:
            transition_to_logging();
            break;
        case STATE_LOGGING:
            transition_to_idle(false);
            break;
        case STATE_SYNCING:
            transition_to_idle(false);
            break;
    }
}

static void handle_button_hold(void) {
    switch (current_state) {
        case STATE_IDLE:
        case STATE_LOGGING:
            transition_to_syncing();
            break;
        case STATE_SYNCING:
            break;
    }
}

// ============== Initialization ==============
static void init_gpio(void) {
    // Configure button
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    
    // Configure status LEDs on GPIO 40, 41, 42
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << POWER_LED_GPIO) | (1ULL << STATUS_LED_GPIO) | (1ULL << ERROR_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_cfg);
    
    // Power LED (GPIO 40) - always on
    gpio_set_level(POWER_LED_GPIO, 1);
    // Status LED (GPIO 41) - off initially (ON=logging, BLINK=syncing)
    gpio_set_level(STATUS_LED_GPIO, 0);
    // Error LED (GPIO 42) - off initially (turns on when errors occur)
    gpio_set_level(ERROR_LED_GPIO, 0);
    ESP_LOGI(TAG, "LEDs initialized: Power=GPIO%d (ON), Status=GPIO%d (OFF), Error=GPIO%d (OFF)", 
             POWER_LED_GPIO, STATUS_LED_GPIO, ERROR_LED_GPIO);
    
    // Configure LED (only if enabled)
#if LED_ENABLED
    ESP_LOGI(TAG, "Initializing NeoPixel LED strip on GPIO%d...", LED_GPIO);
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(ret));
        led_strip = NULL;
    } else {
        vTaskDelay(pdMS_TO_TICKS(10));
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "NeoPixel LED strip initialized on GPIO%d", LED_GPIO);
    }
    ESP_LOGI(TAG, "GPIO initialized: Button=GPIO%d (BOOT), LED=GPIO%d", BUTTON_GPIO, LED_GPIO);
#else
    ESP_LOGI(TAG, "GPIO initialized: Button=GPIO%d (BOOT), LED=disabled (GPIO%d reserved for SD CS)", BUTTON_GPIO, LED_GPIO);
#endif
}

static void init_bno055_address_pin(void) {
    gpio_config_t addr_cfg = {
        .pin_bit_mask = (1ULL << BNO055_ADDR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&addr_cfg);
    gpio_set_level(BNO055_ADDR_GPIO, 0);  // LOW = address 0x28
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "BNO055 address pin (GPIO%d) set LOW for 0x28", BNO055_ADDR_GPIO);
}

// ============== Main Application ==============
void app_main(void) {
    ESP_LOGI(TAG, "GoldenForm Firmware (ESP32-S3-WROOM-1 starting)");

    // Initialize GPIO (button, LED)
    init_gpio();
    
    // Initialize BNO055 address selection
    init_bno055_address_pin();

    // Initialize I2C
    esp_err_t err = bus_i2c_init(I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO, 100000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2C initialized: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
    }

    // Initialize BNO055 - retry up to 5 times per TIDR 1-3-1
    {
        const int IMU_MAX_RETRIES = 5;
        for (int attempt = 1; attempt <= IMU_MAX_RETRIES; attempt++) {
            ESP_LOGI(TAG, "IMU init attempt %d/%d", attempt, IMU_MAX_RETRIES);
            err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
            if (err == ESP_OK) {
                bno055_available = true;
                ESP_LOGI(TAG, "BNO055 initialized at 0x%02X (attempt %d)", BNO055_ADDR_A, attempt);
                break;
            }
            ESP_LOGW(TAG, "IMU init attempt %d failed: %s", attempt, esp_err_to_name(err));
            if (attempt < IMU_MAX_RETRIES) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        if (!bno055_available) {
            ESP_LOGE(TAG, "IMU initialization failed after %d attempts - entering error state",
                     IMU_MAX_RETRIES);
            error_led_set(true);
        }
    }

    // Initialize SD card storage
    err = storage_init();
    if (err == ESP_OK) {
        storage_available = true;
        ESP_LOGI(TAG, "SD card storage initialized");
    } else {
        ESP_LOGW(TAG, "SD card not available - continuing without storage");
        error_led_set(true);
    }

    // Initialize WiFi server
    err = wifi_server_init();
    if (err == ESP_OK) {
        wifi_available = true;
        ESP_LOGI(TAG, "WiFi AP initialized: %s", WIFI_AP_SSID);
    } else {
        ESP_LOGW(TAG, "WiFi init failed - continuing without WiFi");
        error_led_set(true);
    }

    // Print status
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "System Status:");
    ESP_LOGI(TAG, "  BNO055 IMU: %s", bno055_available ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  SD Card: %s", storage_available ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  WiFi AP: %s", wifi_available ? "OK" : "FAILED");
    if (system_has_error) {
        ESP_LOGW(TAG, "  Error LED: ON (one or more subsystems failed)");
    }
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "BOOT button: Press = Start/Stop recording");
    ESP_LOGI(TAG, "             Hold  = Sync via WiFi");
    ESP_LOGI(TAG, "==========================================");

    // Ensure LED is OFF in IDLE state
    led_set(false);
    current_state = STATE_IDLE;

    // Main loop - uses vTaskDelayUntil() for precise timing-critical IMU sampling
    // This ensures consistent sample intervals even if other tasks run
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FORMSYNC_SAMPLE_HZ);

    while (1) {
        // Detect button hold (while button is still down)
        if (button_is_pressed && !button_hold_triggered && button_press_start_ms > 0) {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if ((now_ms - button_press_start_ms) >= BUTTON_HOLD_MS) {
                button_hold_triggered = true;
                ESP_LOGI(TAG, "Button HOLD detected");
                handle_button_hold();
            }
        }

        // Handle short press (on release)
        if (button_short_press) {
            button_short_press = false;
            ESP_LOGI(TAG, "Button SHORT PRESS detected");
            handle_button_press();
        }

        // Auto-return to idle when sync transfer completes
        if (current_state == STATE_SYNCING && wifi_available &&
            wifi_server_is_transfer_complete()) {
            ESP_LOGI(TAG, "Transfer complete - auto-returning to IDLE");
            vTaskDelay(pdMS_TO_TICKS(2000));
            transition_to_idle(true);
        }

        // Read IMU whenever sensor is available (for UART live stream + optional SD logging)
        if (bno055_available) {
            bno055_sample_t sample;
            err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &sample);
            if (err == ESP_OK) {
                // Calibration monitoring - log status changes and announce full calibration
                {
                    static int8_t prev_sys = -1, prev_gyro = -1, prev_accel = -1, prev_mag = -1;
                    static bool fully_calibrated_announced = false;
                    bool changed = (sample.sys_cal != prev_sys || sample.gyro_cal != prev_gyro ||
                                    sample.accel_cal != prev_accel || sample.mag_cal != prev_mag);
                    if (changed) {
                        ESP_LOGI("CAL", "Sys:%d/3  Gyro:%d/3  Accel:%d/3  Mag:%d/3",
                                 sample.sys_cal, sample.gyro_cal, sample.accel_cal, sample.mag_cal);
                        prev_sys = sample.sys_cal;
                        prev_gyro = sample.gyro_cal;
                        prev_accel = sample.accel_cal;
                        prev_mag = sample.mag_cal;

                        if (sample.sys_cal == 3 && sample.gyro_cal == 3 &&
                            sample.accel_cal == 3 && sample.mag_cal == 3) {
                            if (!fully_calibrated_announced) {
                                ESP_LOGI("CAL", "========================================");
                                ESP_LOGI("CAL", "  ALL SENSORS FULLY CALIBRATED (3/3)");
                                ESP_LOGI("CAL", "  Session data will have best accuracy");
                                ESP_LOGI("CAL", "========================================");
                                fully_calibrated_announced = true;
                            }
                        } else {
                            fully_calibrated_announced = false;
                        }
                    }
                }

                // Stream JSON to UART for live visualizer (every 2 samples ~50 Hz at 100 Hz rate)
                static uint32_t uart_stream_count = 0;
                uart_stream_count++;
                if (uart_stream_count >= 2) {
                    uart_stream_count = 0;
                    char json_data[512];
                    snprintf(json_data, sizeof(json_data),
                        "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"lia_x\":%.3f,\"lia_y\":%.3f,\"lia_z\":%.3f,\"temp\":%.1f,\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}}",
                        (unsigned) sample.t_ms, sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz,
                        sample.mx, sample.my, sample.mz, sample.roll, sample.pitch, sample.yaw,
                        sample.qw, sample.qx, sample.qy, sample.qz, sample.lia_x, sample.lia_y, sample.lia_z, sample.temp,
                        sample.sys_cal, sample.gyro_cal, sample.accel_cal, sample.mag_cal);
                    // printf("%s\n", json_data);
                }

                // When logging, also store to SD card
                if (current_state == STATE_LOGGING && storage_available && storage_is_recording()) {
                    if (DEBUG_SD_CARD) {
                        sample.ax = 1.0f;
                        sample.ay = 1.0f;
                        sample.az = 1.0f;
                        sample.gx = 1.0f;
                        sample.gy = 1.0f;
                        sample.gz = 1.0f;
                        sample.mx = 1.0f;
                        sample.my = 1.0f;
                        sample.mz = 1.0f;
                        sample.roll = 1.0f;
                        sample.pitch = 1.0f;
                        sample.yaw = 1.0f;
                        sample.qw = 1.0f;
                        sample.qx = 1.0f;
                        sample.qy = 1.0f;
                        sample.qz = 1.0f;
                        sample.lia_x = 1.0f;
                        sample.lia_y = 1.0f;
                        sample.lia_z = 1.0f;
                        sample.temp = 1.0f;
                        sample.sys_cal = 1;
                        sample.gyro_cal = 1;
                        sample.accel_cal = 1;
                        sample.mag_cal = 1;
                    }
                    err = storage_enqueue_bno_sample(&sample);
                    if (err == ESP_OK) {
                        session_sample_count++;
                        if (session_sample_count % 100 == 0) {
                            ESP_LOGI(TAG, "Logging: %u samples", session_sample_count);
                        }
                    }
                }
            }
        }

        // Status update every 5 seconds when idle
        static uint32_t status_counter = 0;
        if (current_state == STATE_IDLE) {
            status_counter++;
            if (status_counter >= (uint32_t)(5 * CONFIG_FORMSYNC_SAMPLE_HZ)) {
                status_counter = 0;
                ESP_LOGI(TAG, "Status: IDLE - Press button to start");
            }
        } else {
            status_counter = 0;  // Reset counter when not idle
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
