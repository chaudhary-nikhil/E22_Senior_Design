/**
 * @file haptic.c
 * @brief Haptic feedback driver for piezoelectric buzzer (LEDC PWM)
 *
 * Uses ESP-IDF LEDC peripheral to generate PWM at the piezo's resonant
 * frequency. Pattern playback runs via a FreeRTOS task so pulses can
 * be multi-step without blocking the main loop.
 */

#include "haptic.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "HAPTIC";

// LEDC configuration
#define HAPTIC_LEDC_TIMER LEDC_TIMER_1
#define HAPTIC_LEDC_CHANNEL LEDC_CHANNEL_1
#define HAPTIC_LEDC_MODE LEDC_LOW_SPEED_MODE
#define HAPTIC_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define HAPTIC_DEFAULT_FREQ_HZ 2700 // Typical piezo resonant frequency
#define HAPTIC_DUTY_50PCT 512       // 50% duty cycle for 10-bit resolution

// State
static bool s_initialized = false;
static bool s_available = false;
static int s_gpio_num = -1;
static esp_timer_handle_t s_pulse_timer = NULL;
static TaskHandle_t s_pattern_task = NULL;
static volatile bool s_pattern_stop = false;

// --- Internal helpers ---

static void haptic_on(uint32_t freq_hz) {
  if (!s_available)
    return;

  // Update frequency
  ledc_set_freq(HAPTIC_LEDC_MODE, HAPTIC_LEDC_TIMER, freq_hz);
  // Set 50% duty (maximum loudness for piezo)
  ledc_set_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL, HAPTIC_DUTY_50PCT);
  ledc_update_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL);
}

static void haptic_off(void) {
  if (!s_available)
    return;
  ledc_set_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL, 0);
  ledc_update_duty(HAPTIC_LEDC_MODE, HAPTIC_LEDC_CHANNEL);
}

// Timer callback for single pulse stop
static void pulse_timer_cb(void *arg) { haptic_off(); }

// Pattern playback task
static void pattern_task(void *arg) {
  haptic_pattern_t pattern = (haptic_pattern_t)(uintptr_t)arg;
  s_pattern_stop = false;

  switch (pattern) {
  case HAPTIC_PATTERN_SINGLE_SHORT:
    haptic_on(HAPTIC_DEFAULT_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(80));
    haptic_off();
    break;

  case HAPTIC_PATTERN_SINGLE_LONG:
    haptic_on(HAPTIC_DEFAULT_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(200));
    haptic_off();
    break;

  case HAPTIC_PATTERN_DOUBLE_PULSE:
    for (int i = 0; i < 2 && !s_pattern_stop; i++) {
      haptic_on(HAPTIC_DEFAULT_FREQ_HZ);
      vTaskDelay(pdMS_TO_TICKS(80));
      haptic_off();
      if (i < 1)
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    break;

  case HAPTIC_PATTERN_TRIPLE_PULSE:
    for (int i = 0; i < 3 && !s_pattern_stop; i++) {
      haptic_on(HAPTIC_DEFAULT_FREQ_HZ);
      vTaskDelay(pdMS_TO_TICKS(50));
      haptic_off();
      if (i < 2)
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    break;

  case HAPTIC_PATTERN_RAMP_UP:
    // Rising frequency from 1500 to 3500 Hz over 300ms
    for (int f = 1500; f <= 3500 && !s_pattern_stop; f += 200) {
      haptic_on((uint32_t)f);
      vTaskDelay(pdMS_TO_TICKS(30));
    }
    haptic_off();
    break;
  }

  haptic_off(); // Ensure off
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
  ESP_LOGI(TAG, "Initializing haptic motor on GPIO%d", gpio_num);

  // Configure LEDC timer
  ledc_timer_config_t timer_cfg = {
      .speed_mode = HAPTIC_LEDC_MODE,
      .timer_num = HAPTIC_LEDC_TIMER,
      .duty_resolution = HAPTIC_LEDC_DUTY_RES,
      .freq_hz = HAPTIC_DEFAULT_FREQ_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  esp_err_t err = ledc_timer_config(&timer_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
    return err;
  }

  // Configure LEDC channel
  ledc_channel_config_t ch_cfg = {
      .speed_mode = HAPTIC_LEDC_MODE,
      .channel = HAPTIC_LEDC_CHANNEL,
      .timer_sel = HAPTIC_LEDC_TIMER,
      .intr_type = LEDC_INTR_DISABLE,
      .gpio_num = gpio_num,
      .duty = 0, // Start off
      .hpoint = 0,
  };
  err = ledc_channel_config(&ch_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
    return err;
  }

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
  ESP_LOGI(TAG, "Haptic motor initialized on GPIO%d (LEDC ch%d, %dHz default)",
           gpio_num, HAPTIC_LEDC_CHANNEL, HAPTIC_DEFAULT_FREQ_HZ);

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

  // Clamp values
  if (duration_ms < 10)
    duration_ms = 10;
  if (duration_ms > 500)
    duration_ms = 500;
  if (freq_hz < 100)
    freq_hz = 100;
  if (freq_hz > 5000)
    freq_hz = 5000;

  ESP_LOGD(TAG, "Pulse: %ums @ %uHz", (unsigned)duration_ms, (unsigned)freq_hz);

  haptic_on(freq_hz);

  // Schedule stop
  if (s_pulse_timer) {
    esp_timer_stop(s_pulse_timer); // Stop any pending timer
    esp_timer_start_once(s_pulse_timer, duration_ms * 1000); // us
  }

  return ESP_OK;
}

esp_err_t haptic_play_pattern(haptic_pattern_t pattern) {
  if (!s_initialized)
    return ESP_ERR_INVALID_STATE;

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
