#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Button configuration
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 2000

// Button states
typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_LONG_PRESSED
} button_state_t;

// Button events
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_PRESSED,
    BUTTON_EVENT_RELEASED,
    BUTTON_EVENT_LONG_PRESSED
} button_event_t;

// Button callback function type
typedef void (*button_callback_t)(button_event_t event);

// Button configuration structure
typedef struct {
    int gpio_num;
    bool active_high;  // true if button is active high, false if active low
    uint32_t debounce_ms;
    uint32_t long_press_ms;
    button_callback_t callback;
} button_config_t;

// Button functions
esp_err_t button_init(const button_config_t* config);
esp_err_t button_deinit(void);
button_state_t button_get_state(void);
button_event_t button_get_event(void);
bool button_is_pressed(void);

#ifdef __cplusplus
}
#endif
