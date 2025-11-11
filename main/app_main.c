#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "wifi_server.h"

static const char *TAG = "GOLDENFORM";

// Button GPIO definitions - Single button for start/stop toggle
#define TOGGLE_BUTTON_GPIO  0   // GPIO0 for toggle logging (BOOT button on ESP32 DevKit C)
#define STATUS_LED_GPIO     2   // LED (GPIO2 - standard for ESP32 DevKit C onboard LED)

// Logging state
static volatile bool logging_active = false;
static volatile bool button_pressed = false;
static volatile uint32_t last_button_time = 0;
#define BUTTON_DEBOUNCE_MS 300

// Session tracking
static uint32_t session_data_count = 0;
static uint32_t session_start_time = 0;

// WiFi server callbacks
static void wifi_status_callback(wifi_server_state_t state) {
    ESP_LOGI(TAG, "WiFi server state changed to: %d", state);
}

static void session_status_callback(session_state_t state) {
    ESP_LOGI(TAG, "Session state changed to: %d", state);
}

// Button interrupt handler with debouncing
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (current_time - last_button_time > BUTTON_DEBOUNCE_MS) {
        button_pressed = true;
        last_button_time = current_time;
    }
}

// Initialize buttons and LED
static esp_err_t init_buttons(void) {
    // Configure TOGGLE button (GPIO0) - BOOT button on ESP32 DevKit C
    gpio_config_t toggle_config = {
        .pin_bit_mask = (1ULL << TOGGLE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Trigger on button press (falling edge)
    };
    ESP_ERROR_CHECK(gpio_config(&toggle_config));
    
    // Configure status LED (GPIO2)
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_config));
    
    // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Add ISR handler for toggle button
    ESP_ERROR_CHECK(gpio_isr_handler_add(TOGGLE_BUTTON_GPIO, button_isr_handler, (void*) TOGGLE_BUTTON_GPIO));
    
    // Initialize LED to OFF
    gpio_set_level(STATUS_LED_GPIO, 0);
    
    ESP_LOGI(TAG, "Button initialized: GPIO%d=Toggle Start/Stop (BOOT button), GPIO%d=LED", 
             TOGGLE_BUTTON_GPIO, STATUS_LED_GPIO);
    return ESP_OK;
}

// Update LED status - ONLY ON during active logging sessions
static void update_led_status(void) {
    gpio_set_level(STATUS_LED_GPIO, logging_active ? 1 : 0);
    ESP_LOGI(TAG, "LED status: %s", logging_active ? "ON (logging)" : "OFF (stopped)");
}

// Toggle logging session (start/stop with single button)
static void toggle_logging_session(void) {
    if (!logging_active) {
        // Start logging
        logging_active = true;
        session_data_count = 0;
        session_start_time = esp_timer_get_time() / 1000; // Convert to milliseconds
        
        ESP_LOGI(TAG, "🟢 LOGGING SESSION STARTED");
        ESP_LOGI(TAG, "📊 Press BOOT button again to STOP logging");
        ESP_LOGI(TAG, "💾 Data will be stored as protobuf on ESP32");
        ESP_LOGI(TAG, "📡 Data will be sent via WiFi to dashboard");
        ESP_LOGI(TAG, "💡 LED is ON - logging is active");
        
        // Start WiFi server logging
        esp_err_t wifi_err = wifi_server_start_logging();
        if (wifi_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start WiFi server logging: %s", esp_err_to_name(wifi_err));
        }
        
        update_led_status();
    } else {
        // Stop logging
        logging_active = false;
        uint32_t session_duration = (esp_timer_get_time() / 1000) - session_start_time;
        
        ESP_LOGI(TAG, "🔴 LOGGING SESSION STOPPED");
        ESP_LOGI(TAG, "📊 Session duration: %u ms", session_duration);
        ESP_LOGI(TAG, "📈 Data points collected: %u", session_data_count);
        ESP_LOGI(TAG, "💾 Protobuf data stored on ESP32");
        ESP_LOGI(TAG, "📡 Sending data via WiFi to dashboard...");
        ESP_LOGI(TAG, "💡 LED is OFF - logging stopped");
        
        // Stop WiFi server logging
        esp_err_t wifi_err = wifi_server_stop_logging();
        if (wifi_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to stop WiFi server logging: %s", esp_err_to_name(wifi_err));
        }
        
        // Session data is available via HTTP GET /session_data endpoint
        ESP_LOGI(TAG, "📤 Session data available via HTTP GET /session_data");
        
        update_led_status();
        
        // Reset session counter
        session_data_count = 0;
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "🏊 GoldenForm Production Firmware Starting");
    ESP_LOGI(TAG, "==========================================");

    // Init I2C (SDA=21, SCL=22 @ 100kHz for BNO055 compatibility)
    esp_err_t i2c_err = bus_i2c_init(I2C_NUM_0, 21, 22, 100000);
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(i2c_err));
        ESP_LOGE(TAG, "Please check I2C wiring");
    } else {
        ESP_LOGI(TAG, "✅ I2C initialized successfully");
    }

    // Init BNO055 IMU (with graceful error handling)
    esp_err_t bno_err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
    if (bno_err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 initialization failed: %s", esp_err_to_name(bno_err));
        ESP_LOGE(TAG, "Please check BNO055 wiring: SDA=21, SCL=22, VCC=3.3V, GND=GND");
        ESP_LOGE(TAG, "Continuing without BNO055 - no IMU data will be logged");
        // Don't crash, just continue without IMU
    } else {
        ESP_LOGI(TAG, "✅ BNO055 initialized successfully");
    }

    // Init buttons and LED
    ESP_ERROR_CHECK(init_buttons());

    // Init WiFi server (with error handling)
    esp_err_t wifi_init_status = wifi_server_init(wifi_status_callback, session_status_callback);
    if (wifi_init_status != ESP_OK) {
        ESP_LOGE(TAG, "WiFi server initialization failed: %s", esp_err_to_name(wifi_init_status));
        ESP_LOGE(TAG, "Continuing without WiFi server - logging will work locally only");
        // Don't crash, just continue without WiFi server
    } else {
        ESP_LOGI(TAG, "✅ WiFi server initialized successfully");
        ESP_LOGI(TAG, "🌐 Connect to WiFi: %s (password: %s)", WIFI_AP_SSID, WIFI_AP_PASSWORD);
        ESP_LOGI(TAG, "🌐 Web interface: http://192.168.4.1");
    }

    ESP_LOGI(TAG, "🚀 All systems initialized - Production logging ready");
    ESP_LOGI(TAG, "🔍 UNIQUE TEST ID: ESP32_GOLDENFORM_TEST_2025");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "🔄 Press BOOT button (GPIO%d) to TOGGLE logging (Start/Stop)", TOGGLE_BUTTON_GPIO);
    ESP_LOGI(TAG, "💡 LED indicates logging status (ON=logging, OFF=stopped)");
    ESP_LOGI(TAG, "📊 Data stored as protobuf → WiFi → Dashboard → Visualization");
    ESP_LOGI(TAG, "==========================================");

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz sampling

    while (1) {
        // Check for button presses
        if (button_pressed) {
            button_pressed = false;
            toggle_logging_session();
        }

        // Read IMU data if logging is active and BNO055 is available
        if (logging_active && bno_err == ESP_OK) {
            bno055_sample_t s;
            esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s);
            if (err == ESP_OK) {
                // Debug: Show first data point and periodic updates
                if (session_data_count == 0) {
                    ESP_LOGI(TAG, "🎯 First IMU data: ax=%.3f, ay=%.3f, az=%.3f, qw=%.4f, qx=%.4f, qy=%.4f, qz=%.4f", 
                             s.ax, s.ay, s.az, s.qw, s.qx, s.qy, s.qz);
                }
                
                // Debug: Show quaternion data every 10 points during logging
                if (session_data_count % 10 == 0 && session_data_count > 0) {
                    ESP_LOGI(TAG, "📊 Point %u: qw=%.4f, qx=%.4f, qy=%.4f, qz=%.4f", 
                             session_data_count, s.qw, s.qx, s.qy, s.qz);
                }
                
                // Send data to WiFi server for storage (only if WiFi server is working)
                if (wifi_init_status == ESP_OK) {
                    esp_err_t add_err = wifi_server_add_data_point(
                        s.t_ms,
                        s.ax, s.ay, s.az,
                        s.gx, s.gy, s.gz,
                        s.qw, s.qx, s.qy, s.qz,
                        s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal
                    );
                    if (add_err != ESP_OK) {
                        ESP_LOGW(TAG, "Failed to add data point: %s", esp_err_to_name(add_err));
                    } else {
                        session_data_count++;
                        // Debug: Show data collection every 50 points
                        if (session_data_count % 50 == 0) {
                            ESP_LOGI(TAG, "📊 Collected %u data points", session_data_count);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "WiFi server not available - data not stored");
                }
            } else {
                ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
            }
        }

        // Status message every 5 seconds
        static int status_count = 0;
        if (status_count % 250 == 0) { // Every 5 seconds at 50Hz
            if (wifi_init_status == ESP_OK) {
                ESP_LOGI(TAG, "Status: Logging=%s, WiFi=%d, Data points=%u", 
                         logging_active ? "ON" : "OFF", 
                         wifi_server_get_state(),
                         wifi_server_get_session_count());
            } else {
                ESP_LOGI(TAG, "Status: Logging=%s, WiFi=FAILED, LED=%s", 
                         logging_active ? "ON" : "OFF",
                         gpio_get_level(STATUS_LED_GPIO) ? "ON" : "OFF");
            }
        }
        status_count++;
        
        vTaskDelayUntil(&t0, period);
    }
}
