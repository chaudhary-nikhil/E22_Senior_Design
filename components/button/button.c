#include "button.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_timer.h"

static const char *TAG = "BUTTON";

// Button state
static button_config_t button_config = {0};
static button_state_t current_state = BUTTON_STATE_IDLE;
static button_event_t current_event = BUTTON_EVENT_NONE;
static bool button_initialized = false;
static TimerHandle_t debounce_timer = NULL;
static uint32_t press_start_time = 0;
static bool long_press_triggered = false;

// Internal functions
static void button_task(void* pvParameters);
static void debounce_timer_callback(TimerHandle_t xTimer);
static bool read_button_pin(void);

esp_err_t button_init(const button_config_t* config) {
    if (button_initialized) {
        ESP_LOGW(TAG, "Button already initialized");
        return ESP_OK;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Button config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy configuration
    button_config = *config;
    
    // Configure GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << button_config.gpio_num),
        .pull_down_en = !button_config.active_high,
        .pull_up_en = button_config.active_high,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", button_config.gpio_num, esp_err_to_name(ret));
        return ret;
    }
    
    // Create debounce timer
    debounce_timer = xTimerCreate("button_debounce", 
                                 pdMS_TO_TICKS(button_config.debounce_ms),
                                 pdFALSE, NULL, debounce_timer_callback);
    if (debounce_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create debounce timer");
        return ESP_FAIL;
    }
    
    // Initialize state
    current_state = BUTTON_STATE_IDLE;
    current_event = BUTTON_EVENT_NONE;
    press_start_time = 0;
    long_press_triggered = false;
    
    // Create button monitoring task
    BaseType_t task_ret = xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        xTimerDelete(debounce_timer, 0);
        return ESP_FAIL;
    }
    
    button_initialized = true;
    ESP_LOGI(TAG, "Button initialized on GPIO %d (active %s)", 
             button_config.gpio_num, button_config.active_high ? "high" : "low");
    
    return ESP_OK;
}

esp_err_t button_deinit(void) {
    if (!button_initialized) {
        return ESP_OK;
    }
    
    // Delete timer
    if (debounce_timer != NULL) {
        xTimerDelete(debounce_timer, 0);
        debounce_timer = NULL;
    }
    
    // Note: Task will self-terminate when button_initialized becomes false
    
    button_initialized = false;
    current_state = BUTTON_STATE_IDLE;
    current_event = BUTTON_EVENT_NONE;
    
    ESP_LOGI(TAG, "Button deinitialized");
    return ESP_OK;
}

button_state_t button_get_state(void) {
    return current_state;
}

button_event_t button_get_event(void) {
    button_event_t event = current_event;
    current_event = BUTTON_EVENT_NONE;  // Clear event after reading
    return event;
}

bool button_is_pressed(void) {
    return current_state == BUTTON_STATE_PRESSED || current_state == BUTTON_STATE_LONG_PRESSED;
}

// Internal functions
static void button_task(void* pvParameters) {
    bool last_button_state = false;
    bool current_button_state = false;
    
    ESP_LOGI(TAG, "Button monitoring task started");
    
    while (button_initialized) {
        current_button_state = read_button_pin();
        
        // Detect button press (transition from not pressed to pressed)
        if (current_button_state && !last_button_state) {
            ESP_LOGD(TAG, "Button press detected");
            press_start_time = (uint32_t)(esp_timer_get_time() / 1000ULL);
            long_press_triggered = false;
            
            // Start debounce timer
            xTimerStart(debounce_timer, 0);
        }
        
        // Detect button release (transition from pressed to not pressed)
        if (!current_button_state && last_button_state) {
            ESP_LOGD(TAG, "Button release detected");
            
            // Stop debounce timer
            xTimerStop(debounce_timer, 0);
            
            // If we were in pressed state and not long press, generate release event
            if (current_state == BUTTON_STATE_PRESSED && !long_press_triggered) {
                current_state = BUTTON_STATE_IDLE;
                current_event = BUTTON_EVENT_RELEASED;
                
                if (button_config.callback) {
                    button_config.callback(BUTTON_EVENT_RELEASED);
                }
                
                ESP_LOGI(TAG, "Button released");
            } else if (current_state == BUTTON_STATE_LONG_PRESSED) {
                current_state = BUTTON_STATE_IDLE;
                ESP_LOGI(TAG, "Long press released");
            }
        }
        
        // Check for long press while button is held
        if (current_button_state && current_state == BUTTON_STATE_PRESSED && !long_press_triggered) {
            uint32_t press_duration = (uint32_t)(esp_timer_get_time() / 1000ULL) - press_start_time;
            if (press_duration >= button_config.long_press_ms) {
                current_state = BUTTON_STATE_LONG_PRESSED;
                current_event = BUTTON_EVENT_LONG_PRESSED;
                long_press_triggered = true;
                
                if (button_config.callback) {
                    button_config.callback(BUTTON_EVENT_LONG_PRESSED);
                }
                
                ESP_LOGI(TAG, "Button long press detected");
            }
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz polling
    }
    
    ESP_LOGI(TAG, "Button monitoring task ended");
    vTaskDelete(NULL);
}

static void debounce_timer_callback(TimerHandle_t xTimer) {
    // This callback is called after debounce time has passed
    // If we reach here, the button press was valid (not noise)
    if (current_state == BUTTON_STATE_IDLE) {
        current_state = BUTTON_STATE_PRESSED;
        current_event = BUTTON_EVENT_PRESSED;
        
        if (button_config.callback) {
            button_config.callback(BUTTON_EVENT_PRESSED);
        }
        
        ESP_LOGI(TAG, "Button pressed (debounced)");
    }
}

static bool read_button_pin(void) {
    int level = gpio_get_level(button_config.gpio_num);
    return (level == 1) == button_config.active_high;
}
