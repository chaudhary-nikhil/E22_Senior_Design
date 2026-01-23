/**
 * @file app_main.c
 * @brief GoldenForm Firmware - ESP32-S3-WROOM-1
 * 
 * State Machine:
 *   IDLE    → (button) → LOGGING  : LED ON,  collecting IMU data to SD
 *   LOGGING → (button) → STOPPED  : LED OFF, SD session closed
 *   STOPPED → (button) → SYNCING  : LED BLINK, WiFi serving data
 *   SYNCING → (button) → IDLE     : LED OFF, ready for new session
 * 
 * Pin Assignments (from schematic, configured in Kconfig):
 *   BNO055 I2C:  SDA=GPIO4, SCL=GPIO5, ADDR=GPIO6, INT=GPIO7
 *   SD Card:     MOSI=GPIO47, MISO=GPIO14, SCK=GPIO21, CS=GPIO48
 *   LEDs:        LED1=GPIO1, LED2=GPIO2, LED3=GPIO8 (not connected yet)
 *   Button:      GPIO0 (BOOT button on module)
 *   Haptic:      GPIO38 (not implemented yet)
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "storage.h"
#include "wifi_server.h"

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

// LED: Using onboard LED (GPIO2) temporarily until PCB LEDs connected
// TODO: Switch to CONFIG_FORMSYNC_LED_1_GPIO when PCB LEDs ready
#define LED_GPIO            2

// ============== Application State Machine ==============
typedef enum {
    STATE_IDLE,         // Not doing anything
    STATE_LOGGING,      // Recording IMU data to SD card
    STATE_STOPPED,      // Recording stopped, data on SD
    STATE_SYNCING       // WiFi active, serving data
} app_state_t;

static volatile app_state_t current_state = STATE_IDLE;
static volatile bool button_pressed = false;
static volatile uint32_t last_button_time = 0;
#define BUTTON_DEBOUNCE_MS 300

// Session stats
static uint32_t session_sample_count = 0;
static uint32_t session_start_time = 0;

// Subsystem status
static bool bno055_available = false;
static bool storage_available = false;
static bool wifi_available = false;

// LED blink task handle
static TaskHandle_t led_blink_task_handle = NULL;
static volatile bool led_blink_stop = false;

// ============== Button ISR ==============
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t now = esp_timer_get_time() / 1000;
    if (now - last_button_time > BUTTON_DEBOUNCE_MS) {
        button_pressed = true;
        last_button_time = now;
    }
}

// ============== LED Control ==============
static void led_set(bool on) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
    ESP_LOGD(TAG, "LED %s", on ? "ON" : "OFF");
}

static void led_blink_task(void *arg) {
    ESP_LOGI(TAG, "LED blink task started");
    led_blink_stop = false;
    
    while (!led_blink_stop) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        if (led_blink_stop) break;
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    
    gpio_set_level(LED_GPIO, 0);  // Ensure LED is off when stopping
    ESP_LOGI(TAG, "LED blink task stopped");
    led_blink_task_handle = NULL;
    vTaskDelete(NULL);
}

static void led_blink_start(void) {
    if (led_blink_task_handle == NULL) {
        led_blink_stop = false;
        // Increased stack size to 2048 for safety
        xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, &led_blink_task_handle);
    }
}

static void led_blink_stop_and_wait(void) {
    if (led_blink_task_handle != NULL) {
        led_blink_stop = true;
        // Wait for task to finish (max 1 second)
        for (int i = 0; i < 20 && led_blink_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    gpio_set_level(LED_GPIO, 0);
}

// ============== State Transitions ==============
static void transition_to_logging(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: LOGGING");
    ESP_LOGI(TAG, "Recording IMU data to SD card...");
    
    session_sample_count = 0;
    session_start_time = esp_timer_get_time() / 1000;
    
    // Ensure any previous blink task is stopped
    led_blink_stop_and_wait();
    
    // Start SD card session
    if (storage_available) {
        esp_err_t err = storage_start_session();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start storage session: %s", esp_err_to_name(err));
        }
    }
    
    // Turn LED ON solid - logging in progress
    led_set(true);
    current_state = STATE_LOGGING;
    
    ESP_LOGI(TAG, "LED ON (GPIO%d) - Recording - Press button to STOP", LED_GPIO);
    ESP_LOGI(TAG, "========================================");
}

static void transition_to_stopped(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: STOPPED");
    
    uint32_t duration_ms = (esp_timer_get_time() / 1000) - session_start_time;
    ESP_LOGI(TAG, "Session complete:");
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms", duration_ms);
    ESP_LOGI(TAG, "  Samples: %" PRIu32, session_sample_count);
    
    // Stop SD card session
    if (storage_available) {
        storage_stop_session();
    }
    
    // Turn LED OFF - stopped
    led_set(false);
    current_state = STATE_STOPPED;
    
    ESP_LOGI(TAG, "LED OFF (GPIO%d) - Press button to SYNC via WiFi", LED_GPIO);
    ESP_LOGI(TAG, "========================================");
}

static void transition_to_syncing(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: SYNCING");
    ESP_LOGI(TAG, "Preparing data for WiFi transfer...");
    
    current_state = STATE_SYNCING;
    
    // Start LED blink to indicate syncing mode
    led_blink_start();
    
    // Start WiFi sync (loads data from SD card into PSRAM buffer)
    if (wifi_available) {
        esp_err_t err = wifi_server_start_sync();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start sync: %s", esp_err_to_name(err));
            ESP_LOGI(TAG, "WiFi server still running - visit http://192.168.4.1");
        } else {
            ESP_LOGI(TAG, "Data loaded into PSRAM buffer");
            ESP_LOGI(TAG, "Connect to WiFi: %s (password: %s)", WIFI_AP_SSID, WIFI_AP_PASSWORD);
            ESP_LOGI(TAG, "Open browser: http://192.168.4.1");
            ESP_LOGI(TAG, "View JSON data at: http://192.168.4.1/data.json");
        }
    }
    
    ESP_LOGI(TAG, "LED BLINKING - Press button to return to IDLE");
    ESP_LOGI(TAG, "========================================");
}

static void transition_to_idle(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, ">>> STATE: IDLE");
    
    // Stop LED blink task first
    led_blink_stop_and_wait();
    
    // Stop WiFi sync and free PSRAM buffer
    if (wifi_available) {
        wifi_server_stop_sync();
    }
    
    led_set(false);
    current_state = STATE_IDLE;
    session_sample_count = 0;
    
    ESP_LOGI(TAG, "LED OFF - Press button to start LOGGING");
    ESP_LOGI(TAG, "========================================");
}

// ============== Button Handler ==============
static void handle_button_press(void) {
    switch (current_state) {
        case STATE_IDLE:
            transition_to_logging();
            break;
        case STATE_LOGGING:
            transition_to_stopped();
            break;
        case STATE_STOPPED:
            transition_to_syncing();
            break;
        case STATE_SYNCING:
            transition_to_idle();
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
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    
    // Configure LED
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_cfg);
    
    // Quick LED test - blink twice to show it's working
    ESP_LOGI(TAG, "Testing LED on GPIO%d...", LED_GPIO);
    for (int i = 0; i < 2; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "GPIO initialized: Button=GPIO%d, LED=GPIO%d", BUTTON_GPIO, LED_GPIO);
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
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "GoldenForm Firmware (ESP32-S3-WROOM-1)");
    ESP_LOGI(TAG, "==========================================");

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

    // Initialize BNO055
    err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
    if (err == ESP_OK) {
        bno055_available = true;
        ESP_LOGI(TAG, "BNO055 initialized at 0x%02X", BNO055_ADDR_A);
    } else {
        ESP_LOGW(TAG, "BNO055 not found - continuing without IMU");
    }

    // Initialize SD card storage
    err = storage_init();
    if (err == ESP_OK) {
        storage_available = true;
        ESP_LOGI(TAG, "SD card storage initialized");
    } else {
        ESP_LOGW(TAG, "SD card not available - continuing without storage");
    }

    // Initialize WiFi server
    err = wifi_server_init();
    if (err == ESP_OK) {
        wifi_available = true;
        ESP_LOGI(TAG, "WiFi AP initialized: %s", WIFI_AP_SSID);
    } else {
        ESP_LOGW(TAG, "WiFi init failed - continuing without WiFi");
    }

    // Print status
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "System Status:");
    ESP_LOGI(TAG, "  BNO055 IMU: %s", bno055_available ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  SD Card: %s", storage_available ? "OK" : "NOT FOUND");
    ESP_LOGI(TAG, "  WiFi AP: %s", wifi_available ? "OK" : "FAILED");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Press BOOT button to start LOGGING");
    ESP_LOGI(TAG, "==========================================");

    // Main loop
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_FORMSYNC_SAMPLE_HZ);
    
    while (1) {
        // Handle button press
        if (button_pressed) {
            button_pressed = false;
            handle_button_press();
        }

        // Collect IMU data if logging
        if (current_state == STATE_LOGGING && bno055_available) {
            bno055_sample_t sample;
            err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &sample);
            
            if (err == ESP_OK) {
                // Store to SD card
                if (storage_available && storage_is_recording()) {
                    err = storage_enqueue_bno_sample(&sample);
                    if (err == ESP_OK) {
                        session_sample_count++;
                        
                        // Log progress every 100 samples
                        if (session_sample_count % 100 == 0) {
                            ESP_LOGI(TAG, "Logging: %u samples", session_sample_count);
                        }
                    }
                }
            }
        }

        // Status update every 5 seconds when idle
        static uint32_t status_counter = 0;
        if (current_state == STATE_IDLE && ++status_counter >= (5 * CONFIG_FORMSYNC_SAMPLE_HZ)) {
            status_counter = 0;
            ESP_LOGI(TAG, "Status: IDLE - Press button to start");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
