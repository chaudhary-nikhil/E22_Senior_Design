#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file dtw_classifier.h
 * @brief Dynamic Time Warping (DTW) for stroke classification
 *
 * Implements a lightweight DTW algorithm optimized for the ESP32 to compare
 * a live 3D LIA (Linear Acceleration) sequence against an ideal reference.
 */

/**
 * @brief Compute the DTW distance between a current sequence and an ideal
 * sequence
 *
 * @param current Array of floats [x0, y0, z0, x1, y1, z1, ...]
 * @param cur_count Number of 3D samples in current
 * @param ideal Array of floats [x0, y0, z0, x1, y1, z1, ...]
 * @param ideal_count Number of 3D samples in ideal
 * @return float Normalized DTW distance (lower is better, 0 = perfect match)
 */
float process_dtw_buffer(const float *current, int cur_count,
                         const float *ideal, int ideal_count);

#ifdef __cplusplus
}
#endif
