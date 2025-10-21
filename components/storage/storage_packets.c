#include "storage.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_timer.h"

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdlib.h>   // malloc, free
#include <unistd.h>   // unlink
#include <stdbool.h>

static const char *TAG = "STORAGE";

// ---------- Storage state ----------
static bool storage_initialized = false;
static bool storage_mounted = false;
static storage_state_t current_state = STORAGE_STATE_IDLE;

static FILE* current_file = NULL;
static char current_filename[STORAGE_MAX_FILENAME_LEN] = {0};
static uint32_t current_file_samples = 0;
static uint32_t current_file_size = 0;
static uint32_t current_session_start = 0;

// ---------- File rollover state ----------
static uint64_t current_file_open_ms = 0;                 // when current file was opened
static const uint32_t FILE_ROLLOVER_MS = 60 * 1000;       // 1 minute
static uint32_t file_index_in_session = 0;                // 0,1,2,...

// ---------- Mount point and card ----------
static const char mount_point[] = "/sdcard";
static sdmmc_card_t* card = NULL;

// ---------- Internal functions ----------
static esp_err_t mount_sd_card(void);
static esp_err_t unmount_sd_card(void);
static esp_err_t create_csv_header(FILE* file);
static esp_err_t open_new_csv_file(void);  // helper for start + rollover

// =======================================================
// Public API
// =======================================================

esp_err_t storage_init(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (storage_initialized) {
        ESP_LOGW(TAG, "Storage already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing storage system...");

    // Mount SD card
    esp_err_t ret = mount_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }

    storage_mounted = true;
    storage_initialized = true;
    current_state = STORAGE_STATE_IDLE;

    ESP_LOGI(TAG, "Storage system initialized successfully");
    return ESP_OK;
#else
    ESP_LOGI(TAG, "Storage disabled in configuration");
    return ESP_OK;
#endif
}

esp_err_t storage_deinit(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (!storage_initialized) {
        return ESP_OK;
    }

    // Stop current session (closes file if open)
    storage_stop_session();

    // Unmount SD card
    esp_err_t ret = unmount_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }

    storage_mounted = false;
    storage_initialized = false;
    current_state = STORAGE_STATE_IDLE;

    ESP_LOGI(TAG, "Storage system deinitialized");
    return ret;
#else
    return ESP_OK;
#endif
}

bool storage_is_available(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    return storage_initialized && storage_mounted && (current_state != STORAGE_STATE_ERROR);
#else
    return false;
#endif
}

storage_state_t storage_get_state(void) {
    return current_state;
}

esp_err_t storage_start_session(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (!storage_is_available()) {
        ESP_LOGE(TAG, "Storage not available");
        return ESP_ERR_INVALID_STATE;
    }
    if (current_state == STORAGE_STATE_RECORDING) {
        ESP_LOGW(TAG, "Already recording");
        return ESP_OK;
    }

    // Make sure we have no stale file handle
    if (current_file) {
        fclose(current_file);
        current_file = NULL;
    }

    // 1) Wipe all old CSVs at the *start* of session
    ESP_LOGI(TAG, "Wiping SD card CSVs at session start...");
    esp_err_t wipe_ret = storage_delete_all_files();
    if (wipe_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wipe old CSVs (continuing anyway)");
    }

    // 2) Reset session counters
    file_index_in_session = 0;
    current_session_start = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // 3) Open first CSV
    esp_err_t ret = open_new_csv_file();
    if (ret != ESP_OK) {
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }

    current_state = STORAGE_STATE_RECORDING;
    ESP_LOGI(TAG, "Started recording session (first file: %s)", current_filename);
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t storage_stop_session(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (current_state != STORAGE_STATE_RECORDING) {
        ESP_LOGW(TAG, "Not currently recording");
        return ESP_OK;
    }

    // Always close the current CSV if open
    if (current_file) {
        fflush(current_file);
        fclose(current_file);
        current_file = NULL;

        uint32_t session_duration = (uint32_t)(esp_timer_get_time() / 1000ULL) - current_session_start;
        ESP_LOGI(TAG, "Stopped recording session");
        ESP_LOGI(TAG, "Last file closed: %s", current_filename);
        ESP_LOGI(TAG, "Session stats - Duration: %u ms, Last file samples: %u, Last file size: %u bytes",
                 session_duration, current_file_samples, current_file_size);
    }

    // Clear per-file meta
    memset(current_filename, 0, sizeof(current_filename));
    current_file_samples = 0;
    current_file_size = 0;
    current_file_open_ms = 0;
    current_session_start = 0;

    current_state = STORAGE_STATE_IDLE;
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

bool storage_is_recording(void) {
    return current_state == STORAGE_STATE_RECORDING;
}

esp_err_t storage_log_imu_sample(uint32_t timestamp, float ax, float ay, float az,
                                 float gx, float gy, float gz, float temp) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (current_state != STORAGE_STATE_RECORDING || current_file == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Check if it’s time to roll the file
    uint64_t now_ms_64 = (esp_timer_get_time() / 1000ULL);
    uint32_t now_ms = (uint32_t)now_ms_64;

    if ((now_ms - current_file_open_ms) >= FILE_ROLLOVER_MS) {
        ESP_LOGI(TAG, "Rollover: closing %s after ~%u ms",
                 current_filename, (uint32_t)(now_ms - current_file_open_ms));

        fflush(current_file);
        fclose(current_file);
        current_file = NULL;

        file_index_in_session++;
        if (open_new_csv_file() != ESP_OK) {
            current_state = STORAGE_STATE_ERROR;
            return ESP_FAIL;
        }
    }

    // 2) Write sample to current file
    int bytes_written = fprintf(current_file, "%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",
                                timestamp, ax, ay, az, gx, gy, gz, temp);
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "Failed to write sample to file");
        current_state = STORAGE_STATE_ERROR;
        return ESP_FAIL;
    }

    // 3) Update counters + periodic flush
    current_file_samples++;
    current_file_size += (uint32_t)bytes_written;

    if ((current_file_samples % 10) == 0) {
        fflush(current_file);
    }

    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t storage_get_current_filename(char* filename, size_t max_len) {
    if (filename == NULL || max_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (current_state != STORAGE_STATE_RECORDING) {
        filename[0] = '\0';
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(filename, current_filename, max_len - 1);
    filename[max_len - 1] = '\0';
    return ESP_OK;
}

esp_err_t storage_get_current_file_size(uint32_t* size) {
    if (size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *size = current_file_size;
    return ESP_OK;
}

esp_err_t storage_get_current_sample_count(uint32_t* count) {
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = current_file_samples;
    return ESP_OK;
}

esp_err_t storage_delete_all_files(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (!storage_is_available()) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deleting all CSV files...");

    // Stop current session if somehow active
    if (current_state == STORAGE_STATE_RECORDING) {
        storage_stop_session();
    }

    DIR* dir = opendir(mount_point);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", mount_point);
        return ESP_FAIL;
    }

    struct dirent* entry;
    uint32_t deleted_count = 0;

    while ((entry = readdir(dir)) != NULL) {
#if defined(DT_REG)
        bool is_reg = (entry->d_type == DT_REG);
#else
        bool is_reg = true; // Some libcs don't set d_type; assume regular
#endif
        if (is_reg && strstr(entry->d_name, ".csv") != NULL) {
            char full_path[128];
            snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, entry->d_name);

            if (unlink(full_path) == 0) {
                deleted_count++;
                ESP_LOGI(TAG, "Deleted: %s", entry->d_name);
            } else {
                ESP_LOGW(TAG, "Failed to delete: %s", entry->d_name);
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Deleted %u CSV files", deleted_count);
    return ESP_OK;
#else
    return ESP_OK;
#endif
}

esp_err_t storage_list_files(char files[][32], uint32_t max_files, uint32_t* actual_count) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    if (!storage_is_available() || files == NULL || actual_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    DIR* dir = opendir(mount_point);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory");
        return ESP_FAIL;
    }

    struct dirent* entry;
    uint32_t count = 0;

    while ((entry = readdir(dir)) != NULL && count < max_files) {
#if defined(DT_REG)
        bool is_reg = (entry->d_type == DT_REG);
#else
        bool is_reg = true;
#endif
        if (is_reg && strstr(entry->d_name, ".csv") != NULL) {
            strncpy(files[count], entry->d_name, 31);
            files[count][31] = '\0';
            count++;
        }
    }

    closedir(dir);
    *actual_count = count;

    return ESP_OK;
#else
    *actual_count = 0;
    return ESP_OK;
#endif
}

// =======================================================
// Internal helpers
// =======================================================

static esp_err_t mount_sd_card(void) {
    // SPI bus for SD (SPI2 / HSPI)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 5;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024
    };

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SD card mounted at %s", mount_point);
    }
    return ret;
}

static esp_err_t unmount_sd_card(void) {
    if (card != NULL) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, card);
        card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
        return ret;
    }
    return ESP_OK;
}

static esp_err_t create_csv_header(FILE* file) {
    if (file == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int ret = fprintf(file, "timestamp_ms,ax_mps2,ay_mps2,az_mps2,gx_rads,gy_rads,gz_rads,temp_c\n");
    if (ret < 0) {
        return ESP_FAIL;
    }

    fflush(file);
    return ESP_OK;
}

// Open a fresh CSV file with timestamp + index, write header, and update state
static esp_err_t open_new_csv_file(void) {
    // Close any existing file first (safety)
    if (current_file) {
        fflush(current_file);
        fclose(current_file);
        current_file = NULL;
    }

    // Name pattern: session_<openTimeMs>_<idx>.csv
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    char filename[48];
    // example: session_1739999999_000.csv
    snprintf(filename, sizeof(filename), "session_%u_%03u.csv", now_ms, file_index_in_session);

    char full_path[96];
    snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, filename);

    current_file = fopen(full_path, "w");
    if (!current_file) {
        ESP_LOGE(TAG, "Failed to create file: %s", full_path);
        return ESP_FAIL;
    }

    // CSV header
    esp_err_t ret = create_csv_header(current_file);
    if (ret != ESP_OK) {
        fclose(current_file);
        current_file = NULL;
        return ret;
    }

    // Update bookkeeping
    snprintf(current_filename, sizeof(current_filename), "%s", filename);
    current_file_samples = 0;
    current_file_size = 0;
    current_file_open_ms = now_ms;

    ESP_LOGI(TAG, "Opened new CSV: %s", current_filename);
    return ESP_OK;
}
//CHANGE FILE ROLOVER TO CHANGE WHEN  IT ROLLS OVER ASO THIS CLEARS SD CARD AT START CHANGE THAT 

