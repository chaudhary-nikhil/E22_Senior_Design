#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_clk_tree.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

static const char *TAG = "ESP32_WROOM";

/**
 * ESP32-WROOM Hello World program
 * Demonstrates ESP32-WROOM specific features with FreeRTOS
 */

// GPIO pins for ESP32-WROOM development boards
#define LED_GPIO 2        // Built-in LED on most ESP32-WROOM dev boards
#define BUTTON_GPIO 0     // Boot button (GPIO 0)

void app_main(void)
{
    ESP_LOGI(TAG, "Hello, ESP32-WROOM World!");
    
    // Get and display chip information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: %s with %d CPU cores, WiFi%s%s", 
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "Free heap size: %lu bytes", (unsigned long)esp_get_free_heap_size());
    
    uint32_t cpu_freq_mhz = 0;
    esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX, &cpu_freq_mhz);
    ESP_LOGI(TAG, "CPU frequency: %lu MHz", (unsigned long)(cpu_freq_mhz / 1000000));
    
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Configure LED GPIO
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&led_conf);
    
    // Configure Button GPIO (with internal pull-up)
    gpio_config_t button_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1,  // Enable internal pull-up
    };
    gpio_config(&button_conf);
    
    ESP_LOGI(TAG, "GPIO configured - LED on GPIO %d, Button on GPIO %d", LED_GPIO, BUTTON_GPIO);
    
    // Main application loop
    uint32_t counter = 0;
    bool button_pressed = false;
    
    while (1) {
        // Read button state
        bool current_button = !gpio_get_level(BUTTON_GPIO); // Inverted due to pull-up
        if (current_button && !button_pressed) {
            ESP_LOGI(TAG, "Button pressed!");
        }
        button_pressed = current_button;
        
        // Toggle LED
        gpio_set_level(LED_GPIO, counter % 2);
        
        // Print status
        ESP_LOGI(TAG, "Counter: %lu, LED: %s, Button: %s", 
                 (unsigned long)counter, 
                 (counter % 2) ? "ON" : "OFF",
                 button_pressed ? "PRESSED" : "RELEASED");
        
        counter++;
        
        // FreeRTOS delay (non-blocking)
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
        
        // Print system info every 10 iterations
        if (counter % 10 == 0) {
            ESP_LOGI(TAG, "Free heap: %lu bytes, Min free heap: %lu bytes", 
                     (unsigned long)esp_get_free_heap_size(), 
                     (unsigned long)esp_get_minimum_free_heap_size());
            ESP_LOGI(TAG, "Uptime: %llu seconds", esp_timer_get_time() / 1000000);
        }
        
        // Demonstrate ESP32-WROOM capabilities
        if (counter % 30 == 0) {
            ESP_LOGI(TAG, "ESP32-WROOM Features:");
            ESP_LOGI(TAG, "  - Dual-core 32-bit processor");
            ESP_LOGI(TAG, "  - Built-in WiFi antenna");
            ESP_LOGI(TAG, "  - Bluetooth Classic & BLE");
            ESP_LOGI(TAG, "  - 4MB Flash, 520KB SRAM");
            ESP_LOGI(TAG, "  - 34 GPIO pins available");
        }
    }
}
