#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "bno055.h"   // defines bno055_sample_t

#ifdef __cplusplus
extern "C" {
#endif

// Storage configuration
#define STORAGE_MAX_FILENAME_LEN 32
#define STORAGE_DEFAULT_FILENAME "imu_data.pb"

// Storage state
typedef enum {
    STORAGE_STATE_IDLE = 0,
    STORAGE_STATE_RECORDING,
    STORAGE_STATE_SD_FULL,      // SD card is full - recording stopped gracefully
    STORAGE_STATE_ERROR
} storage_state_t;

// ============== Core storage functions ==============

/**
 * @brief Initialize the storage system
 * 
 * Mounts SD card, initializes PSRAM ring buffer, and prepares for recording.
 * Must be called before any other storage functions.
 */
esp_err_t storage_init(void);

/**
 * @brief Deinitialize storage and free resources
 */
esp_err_t storage_deinit(void);

/**
 * @brief Check if storage is available and ready
 */
bool storage_is_available(void);

/**
 * @brief Get current storage state
 */
storage_state_t storage_get_state(void);

// ============== Session control ==============

/**
 * @brief Start a recording session
 * 
 * Starts the background flush task and opens a new data file.
 * Clears any previous session data from SD card.
 */
esp_err_t storage_start_session(void);

/**
 * @brief Stop the recording session
 * 
 * Flushes remaining samples from PSRAM buffer to SD card,
 * stops the flush task, and closes the current file.
 */
esp_err_t storage_stop_session(void);

/**
 * @brief Check if currently recording
 */
bool storage_is_recording(void);

// ============== Sample ingestion ==============

/**
 * @brief Push a sample into the PSRAM ring buffer
 * 
 * This is the primary function for adding IMU data during recording.
 * Samples are buffered in PSRAM and periodically flushed to SD card
 * by a background task.
 * 
 * @param sample Pointer to sample to store
 * @return ESP_OK on success
 */
esp_err_t storage_push_sample(const bno055_sample_t *sample);

/**
 * @brief Legacy API - enqueue sample (calls storage_push_sample internally)
 */
esp_err_t storage_enqueue_bno_sample(const bno055_sample_t *sample);

/**
 * @brief Legacy shim for older API
 */
esp_err_t storage_log_imu_sample(uint32_t timestamp,
                                 float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float temp);

// ============== File queries ==============

/**
 * @brief Get the current recording filename
 */
esp_err_t storage_get_current_filename(char *filename, size_t max_len);

/**
 * @brief Get current file size in bytes
 */
esp_err_t storage_get_current_file_size(uint32_t *size);

/**
 * @brief Get current sample count in session
 */
esp_err_t storage_get_current_sample_count(uint32_t *count);

/**
 * @brief Get remaining SD card space in bytes
 */
esp_err_t storage_get_free_space(uint64_t *free_bytes);

// ============== File management ==============

/**
 * @brief Delete all data files from SD card
 */
esp_err_t storage_delete_all_files(void);

/**
 * @brief List all data files on SD card
 */
esp_err_t storage_list_files(char files[][32], uint32_t max_files, uint32_t *actual_count);

/**
 * @brief List only .pb files (protobuf format, also supports legacy .BIN/.bin)
 */
esp_err_t storage_list_bin_files(char files[][32], uint32_t max_files, 
                                  uint32_t *actual_count, bool sort_asc);

// ============== Data reading for WiFi sync ==============

/**
 * @brief Callback type for iterating over samples
 */
typedef void (*storage_samples_cb_t)(const bno055_sample_t *samples,
                                     size_t count, void *ctx);

/**
 * @brief Read a single BIN file into buffer
 */
esp_err_t storage_read_bin_file_into_buffer(const char *filename,
                                            bno055_sample_t *buf,
                                            size_t cap,
                                            size_t *out_len);

/**
 * @brief Iterate over all .pb files with callback (protobuf format)
 */
esp_err_t storage_iterate_bin_files(storage_samples_cb_t cb,
                                    void *ctx,
                                    size_t chunk);

/**
 * @brief Read ALL .pb files into a single buffer (protobuf format, supports legacy .BIN/.bin)
 * 
 * Reads and decodes protobuf data from all session files.
 * Used during WiFi sync to prepare data for transfer.
 */
esp_err_t storage_read_all_bin_into_buffer(bno055_sample_t *out,
                                           size_t out_cap,
                                           size_t *out_len);

// ============== In-RAM recent samples (for optional inspection) ==============

/**
 * @brief Push sample to both PSRAM buffer and recent ring
 * @deprecated Use storage_push_sample instead
 */
esp_err_t storage_push_bno_sample(const bno055_sample_t *s);

/**
 * @brief Copy most recent samples from small in-RAM ring
 */
size_t storage_copy_recent(bno055_sample_t *out, size_t max_out);

// ============== Statistics ==============

/**
 * @brief Storage statistics structure
 */
typedef struct {
    size_t psram_buffer_count;     // Current samples in PSRAM buffer
    size_t psram_buffer_capacity;  // PSRAM buffer max capacity
    size_t total_samples_written;  // Total samples written to SD
    size_t total_samples_dropped;  // Samples dropped due to overflow
    size_t current_file_size;      // Current file size in bytes
    uint64_t sd_free_bytes;        // Free space on SD card
    bool is_recording;
    bool is_sd_full;
} storage_stats_t;

/**
 * @brief Get comprehensive storage statistics
 */
esp_err_t storage_get_stats(storage_stats_t *stats);

/**
 * @brief Get current session number (0 = no sessions recorded yet)
 */
uint32_t storage_get_session_number(void);

/**
 * @brief Reset session counter and delete all files (use after successful sync)
 */
void storage_reset_session_counter(void);

#ifdef __cplusplus
}
#endif
