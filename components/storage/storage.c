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
#include <inttypes.h>   // PRIu32
#include <stdlib.h>     // malloc, free
#include <unistd.h>     // unlink, fsync
#include <stdbool.h>
#include <errno.h>      // strerror

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
static const uint32_t FILE_ROLLOVER_MS = 20 * 1000;       // set 60*1000 for 1 minute
static uint32_t file_index_in_session = 0;                // 0,1,2,...

// ---------- Mount point and card ----------
static const char mount_point[] = "/sdcard";
static sdmmc_card_t* card = NULL;

// ---------- BLE streaming callback type ----------
// A transport-agnostic send callback used by the storage streaming API.
// Return true if bytes were accepted/queued; false to abort streaming.
typedef bool (*storage_ble_send_cb)(const uint8_t *data, size_t len, void *user_ctx);

// ---------- Internal functions (prototypes) ----------
static esp_err_t mount_sd_card(void);
static esp_err_t unmount_sd_card(void);
static esp_err_t create_csv_header(FILE* file);
static esp_err_t open_new_csv_file(void);  // helper for start + rollover
static bool ble_chunk_and_send(const uint8_t *data, size_t len,
                               size_t max_payload,
                               storage_ble_send_cb send_cb, void *ctx);
static esp_err_t list_files_to_text(char *out, size_t out_cap, uint32_t *count_out);

// Public helper prototypes (could be in storage.h if needed)
esp_err_t storage_stream_file_over_ble(const char *filename,
                                       size_t ble_payload_max,
                                       storage_ble_send_cb send_cb,
                                       void *user_ctx);
esp_err_t storage_read_file_into_buffer(const char *filename,
                                        uint8_t *buf, size_t buf_cap,
                                        size_t *out_len);

// =======================================================
// Public API
// =======================================================

/**
 * @brief Initialize the storage subsystem: mounts the SD card and primes internal state.
 */
esp_err_t storage_init(void) {
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
}

/**
 * @brief Deinitialize the storage subsystem: stops session, unmounts the SD card.
 */
esp_err_t storage_deinit(void) {
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
}

/**
 * @brief Indicates whether storage is mounted and not in an error state.
 */
bool storage_is_available(void) {
    return storage_initialized && storage_mounted && (current_state != STORAGE_STATE_ERROR);
}

/**
 * @brief Returns the current storage state (IDLE, RECORDING, ERROR).
 */
storage_state_t storage_get_state(void) {
    return current_state;
}

/**
 * @brief Start a new recording session: optionally clears old CSVs, resets counters, opens first CSV.
 */
esp_err_t storage_start_session(void) {
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
        fflush(current_file);
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
}

/**
 * @brief Stop an active recording session and close the current CSV (if any).
 */
esp_err_t storage_stop_session(void) {
    if (current_state != STORAGE_STATE_RECORDING) {
        ESP_LOGW(TAG, "Not currently recording");
        return ESP_OK;
    }

    // Always close the current CSV if open
    if (current_file) {
        fflush(current_file);
        // Ensure FAT metadata hits the card as well
        fsync(fileno(current_file));
        fclose(current_file);
        current_file = NULL;

        uint32_t session_duration = (uint32_t)(esp_timer_get_time() / 1000ULL) - current_session_start;
        ESP_LOGI(TAG, "Stopped recording session");
        ESP_LOGI(TAG, "Last file closed: %s", current_filename);
        ESP_LOGI(TAG, "Session stats - Duration: %" PRIu32 " ms, Last file samples: %" PRIu32 ", Last file size: %" PRIu32 " bytes",
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
}

/**
 * @brief Returns true if currently in recording mode.
 */
bool storage_is_recording(void) {
    return current_state == STORAGE_STATE_RECORDING;
}

/**
 * @brief Append one IMU sample (CSV row) to the current file; rolls to a new file on timeout.
 */
esp_err_t storage_log_imu_sample(uint32_t timestamp, float ax, float ay, float az,
                                 float gx, float gy, float gz, float temp) {
    if (current_state != STORAGE_STATE_RECORDING || current_file == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // 1) Check if it’s time to roll the file
    uint64_t now_ms_64 = (esp_timer_get_time() / 1000ULL);
    uint32_t now_ms = (uint32_t)now_ms_64;

    if ((now_ms - current_file_open_ms) >= FILE_ROLLOVER_MS) {
        ESP_LOGI(TAG, "Rollover: closing %s after ~%" PRIu32 " ms",
                 current_filename, (uint32_t)(now_ms - current_file_open_ms));

        fflush(current_file);
        fsync(fileno(current_file));
        fclose(current_file);
        current_file = NULL;

        file_index_in_session++;
        if (open_new_csv_file() != ESP_OK) {
            current_state = STORAGE_STATE_ERROR;
            return ESP_FAIL;
        }
    }

    // 2) Write sample to current file
    int bytes_written = fprintf(current_file, "%" PRIu32 ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",
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
        fsync(fileno(current_file));
    }

    return ESP_OK;
}

/**
 * @brief Copy the current CSV filename into the provided buffer.
 */
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

/**
 * @brief Return the byte size written so far in the current file.
 */
esp_err_t storage_get_current_file_size(uint32_t* size) {
    if (size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *size = current_file_size;
    return ESP_OK;
}

/**
 * @brief Return the number of samples appended to the current file.
 */
esp_err_t storage_get_current_sample_count(uint32_t* count) {
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = current_file_samples;
    return ESP_OK;
}

/**
 * @brief Delete all .csv files in the SD card root at mount_point.
 */
esp_err_t storage_delete_all_files(void) {
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
            // Build full path safely with tight bounds
            char full_path[320];                           // enough for "/sdcard/" + 255 + NUL
            const char *prefix = mount_point;              // "/sdcard"
            const size_t prefix_len = strlen(prefix);

            // d_name is 255+1 (NUL) in this toolchain
            size_t dname_cap = sizeof(entry->d_name) - 1;  // 255
            size_t max_for_path = sizeof(full_path) - prefix_len - 2; // '/' + NUL
            size_t max_name = (dname_cap < max_for_path) ? dname_cap : max_for_path;

            size_t name_len = strnlen(entry->d_name, max_name);

            int n = snprintf(full_path, sizeof(full_path), "%s/%.*s",
                             prefix, (int)name_len, entry->d_name);
            if (n < 0 || (size_t)n >= sizeof(full_path)) {
                ESP_LOGW(TAG, "Path truncated, skipping delete");
                continue;
            }

            if (unlink(full_path) == 0) {
                deleted_count++;
                ESP_LOGI(TAG, "Deleted: %s", entry->d_name);
            } else {
                ESP_LOGW(TAG, "Failed to delete: %s", entry->d_name);
            }
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Deleted %" PRIu32 " CSV files", deleted_count);
    return ESP_OK;
}

/**
 * @brief Enumerate .csv files into a fixed array of short names (up to 31 chars each).
 */
esp_err_t storage_list_files(char files[][32], uint32_t max_files, uint32_t* actual_count) {
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
}

// =======================================================
// Internal helpers
// =======================================================

/**
 * @brief Mount the SD card using SPI mode and register it at mount_point.
 */
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

/**
 * @brief Unmount the SD card and release VFS + host resources.
 */
static esp_err_t unmount_sd_card(void) {
    if (card != NULL) {
        esp_err_t ret = esp_vfs_fat_sdcard_unmount(mount_point, card);
        card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
        return ret;
    }
    return ESP_OK;
}

/**
 * @brief Write a single CSV header row to the given file and flush it.
 */
static esp_err_t create_csv_header(FILE* file) {
    if (file == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int ret = fprintf(file, "timestamp_ms,ax_mps2,ay_mps2,az_mps2,gx_rads,gy_rads,gz_rads,temp_c\n");
    if (ret < 0) {
        return ESP_FAIL;
    }

    fflush(file);
    fsync(fileno(file));
    return ESP_OK;
}

/**
 * @brief Open a new CSV file using a short 8.3 timestamped name, write header, update bookkeeping.
 */
static esp_err_t open_new_csv_file(void) {
    // Close any existing file first (safety)
    if (current_file) {
        fflush(current_file);
        fsync(fileno(current_file));
        fclose(current_file);
        current_file = NULL;
    }

    // Name pattern: S<5-digit time><2-digit index>.CSV  -> always 8.3
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t t5 = now_ms % 100000;            // last 5 digits
    uint32_t idx2 = file_index_in_session % 100;

    char filename[13];  // "S12345A7.CSV" fits (<= 12 + NUL)
    snprintf(filename, sizeof(filename), "S%05" PRIu32 "%02" PRIu32 ".CSV", t5, idx2);

    char full_path[96];
    int n = snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(full_path)) {
        ESP_LOGE(TAG, "Full path too long");
        return ESP_FAIL;
    }

    current_file = fopen(full_path, "w");
    if (!current_file) {
        ESP_LOGE(TAG, "Failed to create file: %s (errno=%d, %s)",
                full_path, errno, strerror(errno));
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

// =======================================================
// Added helpers: listing to text + BLE-friendly streaming
// =======================================================

/**
 * @brief Convert the list of CSV filenames into a single newline-separated text buffer.
 *
 * The result is NUL-terminated if space allows. Useful for BLE/UI manifests.
 */
static __attribute__((unused)) esp_err_t list_files_to_text(char *out, size_t out_cap, uint32_t *count_out) { //CHANGE THIS TO USED WHEN USING IN MAIN 
    if (!out || out_cap == 0 || !count_out) return ESP_ERR_INVALID_ARG;
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    DIR* dir = opendir(mount_point);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed", mount_point);
        return ESP_FAIL;
    }

    size_t used = 0;
    uint32_t count = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
#if defined(DT_REG)
        bool is_reg = (entry->d_type == DT_REG);
#else
        bool is_reg = true;
#endif
        if (!is_reg) continue;
        if (strstr(entry->d_name, ".csv") == NULL && strstr(entry->d_name, ".CSV") == NULL) continue;

        size_t name_len = strnlen(entry->d_name, 255);
        if (used + name_len + 1 >= out_cap) break;  // +1 for '\n'

        memcpy(out + used, entry->d_name, name_len);
        used += name_len;
        out[used++] = '\n';
        count++;
    }
    closedir(dir);

    // NUL-terminate if space remains
    if (used < out_cap) out[used] = '\0';
    *count_out = count;
    return ESP_OK;
}

/**
 * @brief Internal helper that slices a buffer into BLE-sized chunks and sends each via callback.
 */
static bool ble_chunk_and_send(const uint8_t *data, size_t len,
                               size_t max_payload,
                               storage_ble_send_cb send_cb, void *ctx) {
    if (!send_cb || max_payload == 0) return false;
    size_t offset = 0;
    while (offset < len) {
        size_t n = len - offset;
        if (n > max_payload) n = max_payload;
        if (!send_cb(data + offset, n, ctx)) {
            return false; // Transport asked us to stop (e.g., back-pressure)
        }
        offset += n;
    }
    return true;
}

/**
 * @brief Stream a file from SD in chunks using a caller-provided send callback (BLE/UART/etc.).
 *
 * A simple text framing is used: a BEGIN line (filename:size) and an END line.
 * Set ble_payload_max to your usable ATT payload (e.g., MTU-3). Returns ESP_OK on success.
 */
esp_err_t storage_stream_file_over_ble(const char *filename,
                                       size_t ble_payload_max,
                                       storage_ble_send_cb send_cb,
                                       void *user_ctx) {
    if (!filename || !send_cb) return ESP_ERR_INVALID_ARG;
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    char path[160];
    int n = snprintf(path, sizeof(path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(path)) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Open failed: %s (errno=%d, %s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }

    // Optional header: BEGIN line with file size
    struct stat st = {0};
    size_t file_size = 0;
    if (stat(path, &st) == 0 && S_ISREG(st.st_mode)) file_size = (size_t)st.st_size;

    char begin_line[256];
    int bl = snprintf(begin_line, sizeof(begin_line), "BEGIN:%s:%u\n", filename, (unsigned)file_size);
    if (bl > 0) {
        if (!ble_chunk_and_send((const uint8_t*)begin_line, (size_t)bl, ble_payload_max, send_cb, user_ctx)) {
            fclose(f);
            return ESP_FAIL;
        }
    }

    // Stream file data in I/O-sized chunks
    uint8_t buf[1024];
    size_t total = 0;

    for (;;) {
        size_t rd = fread(buf, 1, sizeof(buf), f);
        if (rd == 0) {
            if (feof(f)) break;
            ESP_LOGE(TAG, "Read error on %s", path);
            fclose(f);
            return ESP_FAIL;
        }
        total += rd;
        if (!ble_chunk_and_send(buf, rd, ble_payload_max, send_cb, user_ctx)) {
            ESP_LOGW(TAG, "BLE aborted during send");
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);

    // Trailer
    const char *end_line = "END\n";
    if (!ble_chunk_and_send((const uint8_t*)end_line, 4, ble_payload_max, send_cb, user_ctx)) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Streamed %s (%u bytes)", filename, (unsigned)total);
    return ESP_OK;
}

/**
 * @brief Convenience method: read a (small) file fully into caller-provided RAM buffer.
 *
 * For large files prefer storage_stream_file_over_ble() to avoid big allocations.
 */
esp_err_t storage_read_file_into_buffer(const char *filename,
                                        uint8_t *buf, size_t buf_cap,
                                        size_t *out_len) {
    if (!filename || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    char path[160];
    int n = snprintf(path, sizeof(path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(path)) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(path, "rb");
    if (!f) return ESP_FAIL;

    size_t total = fread(buf, 1, buf_cap, f);
    fclose(f);

    *out_len = total;
    // If file size exceeds buf_cap, signal partial read
    if (total == buf_cap) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

//CHANGE FILE ROLOVER TO CHANGE WHEN  IT ROLLS OVER ASO THIS CLEARS SD CARD AT START CHANGE THAT 

