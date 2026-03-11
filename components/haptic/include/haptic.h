#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file haptic.h
 * @brief Haptic feedback driver for ERM vibration motor (Pure GPIO)
 *
 * Drives an ERM (Eccentric Rotating Mass) vibration motor via a simple
 * GPIO digital high/low output. The motor is switched through an 
 * N-channel MOSFET (AO3400A) for current handling.
 *
 * Hardware testing showed PWM caused unreliable gate switching, so
 * patterns use specific ON/OFF temporal pulses instead of intensity control.
 *
 * Hardware: ERM motor on GPIO 38 (CONFIG_GOLDENFORM_HAPTIC_GPIO)
 *   GPIO -> 100Ω -> MOSFET Gate, 10kΩ pull-down to GND
 *   Motor through MOSFET low-side switch with 1N5817 flyback diode
 */

// Haptic pattern types for different alert scenarios
typedef enum {
  HAPTIC_PATTERN_SINGLE_SHORT,    // 80ms buzz — stroke detected
  HAPTIC_PATTERN_SINGLE_LONG,     // 200ms buzz — moderate deviation
  HAPTIC_PATTERN_DOUBLE_PULSE,    // Two 80ms buzzes, 60ms gap — high deviation
  HAPTIC_PATTERN_TRIPLE_PULSE,    // Three 50ms buzzes, 40ms gaps — critical
  HAPTIC_PATTERN_RAMP_UP,         // Stuttering fast pulses — turn/wall alert
  HAPTIC_PATTERN_DEVIATION_MILD,  // 120ms buzz — mild deviation (beginners)
  HAPTIC_PATTERN_DEVIATION_STRONG // 150ms+100ms buzzes — strong deviation
} haptic_pattern_t;

/**
 * @brief Initialize the ERM haptic motor driver
 *
 * Configures LEDC timer (1kHz) and channel for PWM duty-cycle output.
 * Safe to call even when no motor is connected.
 *
 * @param gpio_num GPIO pin connected to MOSFET gate
 * @return ESP_OK on success
 */
esp_err_t haptic_init(int gpio_num);

/**
 * @brief Fire a single haptic pulse at default intensity (70%)
 *
 * Non-blocking: starts the pulse and returns immediately.
 * A FreeRTOS timer callback stops the pulse after duration_ms.
 *
 * @param duration_ms Pulse duration in milliseconds (30-500)
 * @param freq_hz Ignored for ERM (kept for API compatibility)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t haptic_pulse(uint32_t duration_ms, uint32_t freq_hz);

/**
 * @brief Set continuous vibration state
 *
 * @param intensity_pct 0 = off, >0 = full on (hardware lacks PWM intensity capability)
 * @return ESP_OK on success
 */
esp_err_t haptic_set_intensity(uint8_t intensity_pct);

/**
 * @brief Play a predefined haptic pattern
 *
 * Non-blocking: starts the pattern playback in background.
 * Includes cooldown debouncing to prevent motor burnout.
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
