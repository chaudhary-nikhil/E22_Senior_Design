#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file haptic.h
 * @brief Haptic feedback driver for piezoelectric buzzer
 *
 * Drives a piezo buzzer via LEDC PWM on a configurable GPIO pin.
 * Designed for the GoldenForm swim form tracker to provide
 * real-time vibrotactile alerts when stroke deviates from ideal.
 *
 * Hardware: Piezoelectric buzzer on GPIO 38 (CONFIG_GOLDENFORM_HAPTIC_GPIO)
 * The buzzer responds to frequency (not just on/off), so PWM is used.
 */

// Haptic pattern types for different alert scenarios
typedef enum {
    HAPTIC_PATTERN_SINGLE_SHORT,    // 80ms pulse — stroke detected
    HAPTIC_PATTERN_SINGLE_LONG,     // 200ms pulse — moderate deviation
    HAPTIC_PATTERN_DOUBLE_PULSE,    // Two 80ms pulses, 60ms gap — high deviation
    HAPTIC_PATTERN_TRIPLE_PULSE,    // Three 50ms pulses, 40ms gaps — critical deviation
    HAPTIC_PATTERN_RAMP_UP,         // 300ms rising frequency — turn/wall alert
} haptic_pattern_t;

/**
 * @brief Initialize the haptic motor driver
 *
 * Configures LEDC timer and channel for PWM output on the specified GPIO.
 * Safe to call even when no motor is connected — will log status and
 * set availability flag accordingly.
 *
 * @param gpio_num GPIO pin connected to piezo buzzer
 * @return ESP_OK on success
 */
esp_err_t haptic_init(int gpio_num);

/**
 * @brief Fire a single haptic pulse
 *
 * Non-blocking: starts the pulse and returns immediately.
 * A FreeRTOS timer callback stops the pulse after duration_ms.
 *
 * @param duration_ms Pulse duration in milliseconds (10-500)
 * @param freq_hz PWM frequency in Hz (100-5000, optimal ~2700 for piezo)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t haptic_pulse(uint32_t duration_ms, uint32_t freq_hz);

/**
 * @brief Play a predefined haptic pattern
 *
 * Non-blocking: starts the pattern playback in background.
 * If a pattern is already playing, it will be interrupted.
 *
 * @param pattern Pattern type to play
 * @return ESP_OK on success
 */
esp_err_t haptic_play_pattern(haptic_pattern_t pattern);

/**
 * @brief Stop any active haptic output immediately
 */
void haptic_stop(void);

/**
 * @brief Check if the haptic motor was successfully initialized
 * @return true if haptic hardware is available and ready
 */
bool haptic_is_available(void);

#ifdef __cplusplus
}
#endif
