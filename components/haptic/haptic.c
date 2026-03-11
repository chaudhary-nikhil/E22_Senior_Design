/**
 * @file haptic.c
 * @brief Haptic feedback driver for ERM vibration motor (LEDC PWM duty-cycle)
 *
 * Drives an ERM (Eccentric Rotating Mass) vibration motor via LEDC PWM on a
 * configurable GPIO pin. The motor is connected through an N-channel MOSFET
 * (AO3400A) low-side switch with a flyback diode for inductive protection.
 *
 * ERM motors respond to average voltage (duty cycle), NOT frequency like piezo
 * buzzers. We use a fixed 1kHz PWM frequency and modulate the duty cycle
 * (0-100%) to control vibration intensity.
 *
 * Hardware: ERM motor on GPIO 38 (CONFIG_GOLDENFORM_HAPTIC_GPIO)
 *   ESP32 GPIO -> 100Ω -> MOSFET Gate (AO3400A)
 *   Motor(+) -> 3.3V rail
 *   Motor(-) -> MOSFET Drain
 *   MOSFET Source -> GND
 *   Flyback diode (1N5817) across motor terminals
 */

#include "haptic.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "HAPTIC";

// Motor timing constraints
#define ERM_SPINUP_MS 30         // Minimum pulse to overcome inertia
#define ERM_COOLDOWN_MS 100      // Minimum gap between patterns
#define ERM_MIN_PATTERN_GAP_MS 150 // Debounce rapid-fire pattern requests

// State
static bool s_initialized = false;
static bool s_available = false;
static int s_gpio_num = -1;
static esp_timer_handle_t s_pulse_timer = NULL;
static TaskHandle_t s_pattern_task = NULL;
static volatile bool s_pattern_stop = false;
static uint32_t s_last_pattern_time_ms = 0;

static void haptic_on(void) {
  if (!s_available)
    return;
  gpio_set_level(s_gpio_num, 1);
}

static void haptic_off(void) {
  if (!s_available)
    return;
  // Temporary: turn pin full OFF
  gpio_set_level(s_gpio_num, 0);
}

// Timer callback for single pulse stop
static void pulse_timer_cb(void *arg) { haptic_off(); }

// Get current time in ms
static uint32_t now_ms(void) {
  return (uint32_t)(esp_timer_get_time() / 1000);
}

// Check if enough time has passed since last pattern
static bool can_start_pattern(void) {
  return (now_ms() - s_last_pattern_time_ms) >= ERM_MIN_PATTERN_GAP_MS;
}

// Pattern playback task
static void pattern_task(void *arg) {
  haptic_pattern_t pattern = (haptic_pattern_t)(uintptr_t)arg;
  s_pattern_stop = false;

  switch (pattern) {
  case HAPTIC_PATTERN_SINGLE_SHORT:
    // Quick 80ms buzz — stroke detected acknowledgment
    haptic_on();
    vTaskDelay(pdMS_TO_TICKS(80));
    haptic_off();
    break;

  case HAPTIC_PATTERN_SINGLE_LONG:
    // 200ms buzz — moderate deviation alert
    haptic_on();
    vTaskDelay(pdMS_TO_TICKS(200));
    haptic_off();
    break;

  case HAPTIC_PATTERN_DOUBLE_PULSE:
    // Two 80ms pulses, 60ms gap — high deviation
    for (int i = 0; i < 2 && !s_pattern_stop; i++) {
      haptic_on();
      vTaskDelay(pdMS_TO_TICKS(80));
      haptic_off();
      if (i < 1)
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    break;

  case HAPTIC_PATTERN_TRIPLE_PULSE:
    // Three 50ms pulses, 40ms gaps — critical deviation
    for (int i = 0; i < 3 && !s_pattern_stop; i++) {
      haptic_on();
      vTaskDelay(pdMS_TO_TICKS(50));
      haptic_off();
      if (i < 2)
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    break;

  case HAPTIC_PATTERN_RAMP_UP:
    // Fast stuttering pulses ending in a long pulse — turn/wall alert
    for (int i = 0; i < 3 && !s_pattern_stop; i++) {
      haptic_on();
      vTaskDelay(pdMS_TO_TICKS(20));
      haptic_off();
      vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (!s_pattern_stop) {
      haptic_on();
      vTaskDelay(pdMS_TO_TICKS(150));
      haptic_off();
    }
    break;

  case HAPTIC_PATTERN_DEVIATION_MILD:
    // Gentle 120ms buzz — mild deviation (beginner-friendly)
    haptic_on();
    vTaskDelay(pdMS_TO_TICKS(120));
    haptic_off();
    break;

  case HAPTIC_PATTERN_DEVIATION_STRONG:
    // Strong 150ms, pause, 100ms — strong deviation
    haptic_on();
    vTaskDelay(pdMS_TO_TICKS(150));
    haptic_off();
    vTaskDelay(pdMS_TO_TICKS(50));
    haptic_on();
    vTaskDelay(pdMS_TO_TICKS(100));
    haptic_off();
    break;
  }

  haptic_off(); // Ensure off
  s_last_pattern_time_ms = now_ms();
  s_pattern_task = NULL;
  vTaskDelete(NULL);
}

// --- Public API ---

esp_err_t haptic_init(int gpio_num) {
  if (s_initialized) {
    ESP_LOGW(TAG, "Already initialized");
    return ESP_OK;
  }

  s_gpio_num = gpio_num;
  ESP_LOGI(TAG, "Initializing ERM haptic motor on GPIO%d", gpio_num);

  // Temporary override: Configure as simple GPIO output instead of LEDC PWM to test the LED
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << gpio_num);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "GPIO config failed for debugging");
      return err;
  }
  gpio_set_level(gpio_num, 0);

  // Create one-shot timer for pulse stop
  esp_timer_create_args_t timer_args = {
      .callback = pulse_timer_cb,
      .name = "haptic_pulse",
  };
  err = esp_timer_create(&timer_args, &s_pulse_timer);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Pulse timer creation failed: %s (patterns still work)",
             esp_err_to_name(err));
    // Non-fatal — patterns use task delay instead
  }

  s_initialized = true;
  s_available = true;
  ESP_LOGI(TAG, "ERM haptic motor initialized on GPIO%d (Pure GPIO Control)", gpio_num);

  return ESP_OK;
}

esp_err_t haptic_pulse(uint32_t duration_ms, uint32_t freq_hz) {
  if (!s_initialized) {
    ESP_LOGW(TAG, "Not initialized — pulse ignored");
    return ESP_ERR_INVALID_STATE;
  }
  if (!s_available) {
    ESP_LOGD(TAG, "Motor not available — pulse logged only");
    return ESP_OK;
  }

  // Clamp duration (freq_hz parameter is ignored for ERM — kept for API compat)
  if (duration_ms < ERM_SPINUP_MS)
    duration_ms = ERM_SPINUP_MS;
  if (duration_ms > 500)
    duration_ms = 500;

  ESP_LOGD(TAG, "Pulse: %ums", (unsigned)duration_ms);

  haptic_on(); // Turn on motor

  // Schedule stop
  if (s_pulse_timer) {
    esp_timer_stop(s_pulse_timer); // Stop any pending timer
    esp_timer_start_once(s_pulse_timer, duration_ms * 1000); // us
  }

  return ESP_OK;
}

esp_err_t haptic_set_intensity(uint8_t intensity_pct) {
  if (!s_initialized)
    return ESP_ERR_INVALID_STATE;

  if (intensity_pct == 0) {
    haptic_off();
  } else {
    haptic_on();
  }
  return ESP_OK;
}

esp_err_t haptic_play_pattern(haptic_pattern_t pattern) {
  if (!s_initialized)
    return ESP_ERR_INVALID_STATE;

  // Debounce rapid-fire pattern requests to prevent motor burnout
  if (!can_start_pattern()) {
    ESP_LOGD(TAG, "Pattern debounced (cooldown)");
    return ESP_OK;
  }

  // Stop any running pattern
  if (s_pattern_task != NULL) {
    s_pattern_stop = true;
    for (int i = 0; i < 20 && s_pattern_task != NULL; i++) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  // Launch pattern task
  xTaskCreate(pattern_task, "haptic_pat", 2048, (void *)(uintptr_t)pattern, 5,
              &s_pattern_task);

  return ESP_OK;
}

void haptic_stop(void) {
  if (s_pulse_timer) {
    esp_timer_stop(s_pulse_timer);
  }
  if (s_pattern_task) {
    s_pattern_stop = true;
  }
  haptic_off();
}

bool haptic_is_available(void) { return s_available; }
