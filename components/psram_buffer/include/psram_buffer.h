#pragma once

#include "esp_err.h"
#include "bno055.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PSRAM Ring Buffer for IMU Samples
 * 
 * Thread-safe circular buffer stored in PSRAM for high-capacity IMU sample
 * collection. Supports configurable watermark-based flush triggers.
 */

// Default configuration values (can be overridden via Kconfig)
#ifndef CONFIG_PSRAM_BUFFER_CAPACITY_SAMPLES
#define CONFIG_PSRAM_BUFFER_CAPACITY_SAMPLES 47662  // Exactly 4MB at 88 bytes/sample (~16 min at 50Hz)
#endif

#ifndef CONFIG_PSRAM_BUFFER_WATERMARK_PERCENT
#define CONFIG_PSRAM_BUFFER_WATERMARK_PERCENT 50    // Trigger flush at 50% full
#endif

#ifndef CONFIG_PSRAM_BUFFER_FLUSH_BATCH_SIZE
#define CONFIG_PSRAM_BUFFER_FLUSH_BATCH_SIZE 256    // Samples per flush batch
#endif

/**
 * @brief Buffer statistics
 */
typedef struct {
    size_t capacity;           // Maximum samples buffer can hold
    size_t count;              // Current number of samples in buffer
    size_t total_pushed;       // Total samples pushed since init/reset
    size_t total_popped;       // Total samples popped since init/reset
    size_t overflows;          // Number of samples dropped due to full buffer
    size_t watermark_threshold;// Threshold for watermark trigger
    bool is_above_watermark;   // Whether buffer is above watermark
} psram_buffer_stats_t;

/**
 * @brief Initialize the PSRAM ring buffer
 * 
 * Allocates memory from PSRAM and initializes the circular buffer.
 * If PSRAM allocation fails, falls back to regular heap (with warning).
 * 
 * @param capacity_samples Maximum number of samples to store (0 = use default)
 * @return ESP_OK on success, ESP_ERR_NO_MEM if allocation fails
 */
esp_err_t psram_buffer_init(size_t capacity_samples);

/**
 * @brief Deinitialize and free the buffer
 */
void psram_buffer_deinit(void);

/**
 * @brief Check if buffer is initialized
 */
bool psram_buffer_is_initialized(void);

/**
 * @brief Push a single sample into the buffer
 * 
 * Thread-safe. If buffer is full, oldest sample is overwritten (circular behavior).
 * 
 * @param sample Pointer to sample to push
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialized
 */
esp_err_t psram_buffer_push(const bno055_sample_t *sample);

/**
 * @brief Push multiple samples into the buffer
 * 
 * @param samples Array of samples to push
 * @param count Number of samples in array
 * @return Number of samples successfully pushed
 */
size_t psram_buffer_push_batch(const bno055_sample_t *samples, size_t count);

/**
 * @brief Pop samples from the buffer (oldest first)
 * 
 * Thread-safe. Removes samples from the buffer and copies to output array.
 * 
 * @param out Output array for samples
 * @param max_samples Maximum samples to pop
 * @return Number of samples actually popped (may be less than max if buffer not full)
 */
size_t psram_buffer_pop_batch(bno055_sample_t *out, size_t max_samples);

/**
 * @brief Peek at samples without removing them
 * 
 * @param out Output array for samples
 * @param max_samples Maximum samples to peek
 * @return Number of samples copied
 */
size_t psram_buffer_peek(bno055_sample_t *out, size_t max_samples);

/**
 * @brief Get current number of samples in buffer
 */
size_t psram_buffer_get_count(void);

/**
 * @brief Check if buffer is above the watermark threshold
 * 
 * Used to trigger flush operations before buffer fills completely.
 */
bool psram_buffer_is_above_watermark(void);

/**
 * @brief Check if buffer is empty
 */
bool psram_buffer_is_empty(void);

/**
 * @brief Check if buffer is full
 */
bool psram_buffer_is_full(void);

/**
 * @brief Get buffer statistics
 * 
 * @param stats Output structure for statistics
 * @return ESP_OK on success
 */
esp_err_t psram_buffer_get_stats(psram_buffer_stats_t *stats);

/**
 * @brief Clear all samples from the buffer
 * 
 * Does not deallocate memory, just resets head/tail pointers.
 */
void psram_buffer_clear(void);

/**
 * @brief Set watermark threshold as percentage of capacity
 * 
 * @param percent Percentage (1-99) at which watermark triggers
 */
void psram_buffer_set_watermark(uint8_t percent);

/**
 * @brief Get the buffer capacity in samples
 */
size_t psram_buffer_get_capacity(void);

#ifdef __cplusplus
}
#endif
