#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   // size_t

#include "bno055.h"   // defines bno055_sample_t


// Storage configuration
#define STORAGE_MAX_FILENAME_LEN 32
#define STORAGE_DEFAULT_FILENAME "imu_data.csv"

// Storage state
typedef enum {
    STORAGE_STATE_IDLE = 0,
    STORAGE_STATE_RECORDING,
    STORAGE_STATE_ERROR
} storage_state_t;

// Core storage functions
esp_err_t       storage_init(void);
esp_err_t       storage_deinit(void);
bool            storage_is_available(void);
storage_state_t storage_get_state(void);

// Session control
esp_err_t storage_start_session(void);
esp_err_t storage_stop_session(void);
bool      storage_is_recording(void);

// Struct-based enqueue (preferred)
esp_err_t storage_enqueue_bno_sample(const bno055_sample_t* s);

// Legacy shim (still supported)
esp_err_t storage_log_imu_sample(uint32_t timestamp,
                                 float ax, float ay, float az,
                                 float gx, float gy, float gz,
                                 float temp);

// File queries
esp_err_t storage_get_current_filename(char* filename, size_t max_len);
esp_err_t storage_get_current_file_size(uint32_t* size);
esp_err_t storage_get_current_sample_count(uint32_t* count);

// Utilities
esp_err_t storage_delete_all_files(void);
esp_err_t storage_list_files(char files[][32], uint32_t max_files, uint32_t* actual_count);

//for read sd part
typedef void (*storage_samples_cb_t)(const bno055_sample_t *samples,
                                     size_t count,
                                     void *ctx);

esp_err_t storage_read_bin_file_into_buffer(const char *filename,
                                            bno055_sample_t *buf,
                                            size_t cap,
                                            size_t *out_len);

esp_err_t storage_iterate_bin_files(storage_samples_cb_t cb,
                                    void *ctx,
                                    size_t chunk /* e.g., 256 */);

esp_err_t storage_list_bin_files(char files[][32],
                                 uint32_t max_files,
                                 uint32_t *actual_count,
                                 bool sort_asc);

  // Push one IMU sample into storage:
//  - copies into a small in-RAM ring (for optional inspection)
//  - enqueues into the SD batch writer (fast, batched fwrite)
esp_err_t storage_push_bno_sample(const bno055_sample_t *s);

// Optional: copy the most recent N samples from the in-RAM ring (no SD touch).
// Returns number of samples copied (<= max_out).
size_t storage_copy_recent(bno055_sample_t *out, size_t max_out);                              

/**
 * @brief Read ALL .BIN files from /sdcard in ascending filename order and copy
 *        bno055_sample_t records into caller buffer.

 * - This assumes each .BIN file is a raw stream of bno055_sample_t structs
 *   written by your logger (same struct layout).
 * - If a file's size isn't a multiple of sizeof(bno055_sample_t), trailing
 *   bytes are ignored and a warning is logged.
 */
esp_err_t storage_read_all_bin_into_buffer(bno055_sample_t *out,
                                           size_t out_cap,
                                           size_t *out_len);

// =======================================================
// SD Card to Serial Streaming (for wireless transfer)
// =======================================================

/**
 * @brief Stream all SD card BIN data to serial as JSON.
 *        Data is sent in same format as live IMU data so web app can receive it.
 *        Includes start/end markers with {"type":"sd_dump"} messages.
 * 
 * @param delay_ms Delay between samples in ms (0 = fast dump, 1-10 = throttled)
 * @return ESP_OK on success
 */
esp_err_t storage_stream_to_serial(uint32_t delay_ms);

/**
 * @brief Get total sample count across all BIN files on SD card.
 * @param count Output: total number of samples
 * @return ESP_OK on success
 */
esp_err_t storage_get_total_sample_count(uint32_t *count);

#ifdef __cplusplus
}
#endif
