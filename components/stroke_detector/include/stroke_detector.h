#pragma once

#include "bno055.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file stroke_detector.h
 * @brief On-device stroke detection and ideal stroke comparison
 *
 * Lightweight C port of the Python StrokeProcessor's detection logic.
 * Detects swim strokes via water-entry accel spikes and compares
 * the current stroke's LIA profile against a stored ideal reference.
 *
 * Only the detection + comparison runs on-device. Full Kalman filtering,
 * position integration, and visualization remain in the Python dashboard.
 */

// Result flags from processing a single sample
typedef struct {
  bool stroke_detected;  // True on the sample where a new stroke was detected
  bool turn_detected;    // True when a wall/turn impact is detected
  bool haptic_fired;     // True if haptic was triggered this sample
  float deviation_score; // 0.0 = perfect match, 1.0+ = poor match
  float entry_angle;     // Water entry angle (degrees) from quaternion pitch
  uint32_t stroke_count; // Running total of strokes detected
  uint32_t turn_count;   // Running total of turns detected
} stroke_event_t;

// Skill levels for haptic thresholding
typedef enum {
  HAPTIC_SKILL_BEGINNER = 0,
  HAPTIC_SKILL_INTERMEDIATE = 1,
  HAPTIC_SKILL_ADVANCED = 2
} haptic_skill_level_t;

void stroke_detector_init(void);

/**
 * @brief Reset session-specific state
 * 
 * Clears stroke and turn counts, history buffers, and current integration.
 * Preserves ideal stroke reference and user configuration.
 */
void stroke_detector_reset_session(void);

/**
 * @brief Process one IMU sample through the detector
 *
 * Call this for every IMU sample in the main loop. The function detects
 * strokes, compares against ideal data, and optionally triggers haptic.
 *
 * @param sample Pointer to current IMU sample
 * @return stroke_event_t Event flags for this sample
 */
stroke_event_t stroke_detector_feed(const bno055_sample_t *sample);

/**
 * @brief Load ideal stroke reference data
 *
 * Stores a reference stroke as an array of LIA (linear acceleration) samples.
 * Each sample is 3 floats: {lia_x, lia_y, lia_z}.
 * The data is copied internally, so the caller can free the source buffer.
 *
 * @param lia_data Array of floats [lia_x0, lia_y0, lia_z0, lia_x1, ...]
 * @param num_samples Number of 3-float samples (array length / 3)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if too large
 */
esp_err_t stroke_detector_load_ideal(const float *lia_data, size_t num_samples, float ideal_entry_angle);

/**
 * @brief Check if ideal stroke data is loaded
 * @return true if ideal data is available for comparison
 */
bool stroke_detector_has_ideal(void);

/**
 * @brief Get the current stroke count
 */
uint32_t stroke_detector_get_count(void);

/**
 * @brief Get the current turn count
 */
uint32_t stroke_detector_get_turn_count(void);

/**
 * @brief Get the last computed deviation score
 * @return 0.0 if no comparison done yet, otherwise the deviation score
 */
float stroke_detector_get_deviation(void);

/**
 * @brief Set the deviation threshold for haptic trigger
 *
 * When the deviation score exceeds this threshold, haptic fires.
 * Default: 0.5 (moderate tolerance).
 *
 * @param threshold Deviation threshold (0.0-2.0 typical range)
 */
void stroke_detector_set_haptic_threshold(float threshold);
/**
 * @brief Get the currently loaded ideal entry angle
 * @return float entry angle in degrees
 */
float stroke_detector_get_ideal_entry_angle(void);

/**
 * @brief Get the last calculated stroke deviation
 * @return float deviation score
 */
float stroke_detector_get_last_deviation(void);

/**
 * @brief Set user physical parameters for stroke normalization
 *
 * @param wingspan_cm Wingspan in centimeters (used to scale LIA)
 * @param skill_level Skill level (adjusts strictness of haptic thresholds)
 */
void stroke_detector_set_user_params(float wingspan_cm, haptic_skill_level_t skill_level);

/**
 * @brief Enable or disable haptic feedback triggering
 *
 * When disabled, deviation is still computed but haptic won't fire.
 * @param enable true to enable haptic triggering
 */
void stroke_detector_enable_haptic(bool enable);

#ifdef __cplusplus
}
#endif
