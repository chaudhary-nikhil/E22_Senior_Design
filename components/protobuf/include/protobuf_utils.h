#pragma once

#include "esp_err.h"
#include "bno055.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encode a single IMU sample to protobuf format
 * 
 * @param sample Input sample from BNO055
 * @param buffer Output buffer for encoded data
 * @param buffer_size Size of output buffer
 * @param encoded_size Output: actual size of encoded data
 * @return ESP_OK on success, ESP_ERR_NO_MEM if buffer too small
 */
esp_err_t protobuf_encode_sample(const bno055_sample_t *sample,
                                  uint8_t *buffer, size_t buffer_size,
                                  size_t *encoded_size);

/**
 * @brief Encode multiple IMU samples as a batch to protobuf format
 * 
 * @param samples Array of input samples
 * @param sample_count Number of samples in array
 * @param buffer Output buffer for encoded data
 * @param buffer_size Size of output buffer
 * @param encoded_size Output: actual size of encoded data
 * @return ESP_OK on success, ESP_ERR_NO_MEM if buffer too small
 */
esp_err_t protobuf_encode_batch(const bno055_sample_t *samples, size_t sample_count,
                                 uint8_t *buffer, size_t buffer_size,
                                 size_t *encoded_size);

/**
 * @brief Decode a single IMU sample from protobuf format
 * 
 * @param buffer Input buffer with encoded data
 * @param buffer_size Size of input data
 * @param sample Output sample structure
 * @return ESP_OK on success, ESP_FAIL on decode error
 */
esp_err_t protobuf_decode_sample(const uint8_t *buffer, size_t buffer_size,
                                  bno055_sample_t *sample);

/**
 * @brief Decode a batch of IMU samples from protobuf format
 * 
 * @param buffer Input buffer with encoded data
 * @param buffer_size Size of input data
 * @param samples Output array of samples
 * @param max_samples Maximum samples to decode
 * @param decoded_count Output: actual number of decoded samples
 * @return ESP_OK on success, ESP_FAIL on decode error
 */
esp_err_t protobuf_decode_batch(const uint8_t *buffer, size_t buffer_size,
                                 bno055_sample_t *samples, size_t max_samples,
                                 size_t *decoded_count);

/**
 * @brief Get estimated encoded size for a batch of samples
 * 
 * @param sample_count Number of samples
 * @return Estimated buffer size needed (conservative estimate)
 */
size_t protobuf_estimate_batch_size(size_t sample_count);

/**
 * @brief Write length-delimited protobuf message to file
 * 
 * Writes a 4-byte little-endian length prefix followed by the protobuf data.
 * This format allows reading variable-length messages from a file stream.
 * 
 * @param file File pointer (must be open for binary writing)
 * @param samples Array of samples to encode
 * @param sample_count Number of samples
 * @return ESP_OK on success
 */
esp_err_t protobuf_write_delimited(FILE *file, const bno055_sample_t *samples,
                                    size_t sample_count);

/**
 * @brief Read length-delimited protobuf message from file
 * 
 * @param file File pointer (must be open for binary reading)
 * @param samples Output array of samples
 * @param max_samples Maximum samples to read
 * @param decoded_count Output: actual number of decoded samples
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND at EOF, ESP_ERR_NO_MEM if allocation fails
 */
esp_err_t protobuf_read_delimited(FILE *file, bno055_sample_t *samples,
                                   size_t max_samples, size_t *decoded_count);

/**
 * @brief Skip a length-delimited protobuf message in a file
 * 
 * Reads the 4-byte length prefix and seeks past the protobuf data.
 * Used for recovery when protobuf_read_delimited fails with ESP_ERR_NO_MEM.
 * 
 * @param file File pointer (must be positioned at start of 4-byte length prefix)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND at EOF, ESP_FAIL on error
 */
esp_err_t protobuf_skip_delimited(FILE *file);

#ifdef __cplusplus
}
#endif
