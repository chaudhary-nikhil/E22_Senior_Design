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
