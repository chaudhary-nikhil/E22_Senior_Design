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
#include <math.h>       // NAN

// Include your struct (ensure storage.h or a separate bno_types.h declares it)
#include "bno_types.h"  // contains typedef bno055_sample_t { ... }

static const char *TAG = "STORAGE";

// ---------------- New config: batching & format ----------------
#define STORAGE_TO_BINARY   1          // 1 = write .BIN (fast/small), 0 = CSV
#define BATCH_CAP           128        // write out when we have this many
#define FLUSH_INTERVAL_MS   250        // also flush if this much time passes

static bno055_sample_t s_batch[BATCH_CAP];
static size_t          s_batch_len = 0;
static uint32_t        s_last_flush_ms = 0;

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
typedef bool (*storage_ble_send_cb)(const uint8_t *data, size_t len, void *user_ctx);

// ---------- Internal functions (prototypes) ----------
static esp_err_t mount_sd_card(void);
static esp_err_t unmount_sd_card(void);
static esp_err_t create_csv_header(FILE* file);
static esp_err_t open_new_csv_file(void);  // (kept name) opens .BIN or .CSV depending on STORAGE_TO_BINARY
static bool ble_chunk_and_send(const uint8_t *data, size_t len,
                               size_t max_payload,
                               storage_ble_send_cb send_cb, void *ctx);
static esp_err_t list_files_to_text(char *out, size_t out_cap, uint32_t *count_out);

// New: batch writer
static esp_err_t flush_batch_locked(bool force);

// Public helper prototypes (unchanged)
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

bool storage_is_available(void) {
    return storage_initialized && storage_mounted && (current_state != STORAGE_STATE_ERROR);
}

storage_state_t storage_get_state(void) {
    return current_state;
}

esp_err_t storage_start_session(void) {
    if (!storage_is_available()) {
        ESP_LOGE(TAG, "Storage not available");
        return ESP_ERR_INVALID_STATE;
    }
    if (current_state == STORAGE_STATE_RECORDING) {
        ESP_LOGW(TAG, "Already recording");
        return ESP_OK;
    }

    // Close any stale file
    if (current_file) {
        fflush(current_file);
        fclose(current_file);
        current_file = NULL;
    }

    // Wipe old logs at session start (now: CSV and BIN)
    ESP_LOGI(TAG, "Wiping SD card logs at session start...");
    esp_err_t wipe_ret = storage_delete_all_files();
    if (wipe_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wipe old logs (continuing anyway)");
    }

    // Reset session counters
    file_index_in_session = 0;
    current_session_start = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // Open first file
    esp_err_t ret = open_new_csv_file();
    if (ret != ESP_OK) {
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }

    // reset batch
    s_batch_len = 0;
    s_last_flush_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    current_state = STORAGE_STATE_RECORDING;
    ESP_LOGI(TAG, "Started recording session (first file: %s)", current_filename);
    return ESP_OK;
}

esp_err_t storage_stop_session(void) {
    if (current_state != STORAGE_STATE_RECORDING) {
        ESP_LOGW(TAG, "Not currently recording");
        return ESP_OK;
    }

    // Final flush of any pending batch
    (void)flush_batch_locked(true);

    // Close file
    if (current_file) {
        fflush(current_file);
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

    s_batch_len = 0;

    current_state = STORAGE_STATE_IDLE;
    return ESP_OK;
}

bool storage_is_recording(void) {
    return current_state == STORAGE_STATE_RECORDING;
}

// ---------------- NEW: enqueue a full struct and batch-write ----------------

esp_err_t storage_enqueue_bno_sample(const bno055_sample_t* s)
{
    if (current_state != STORAGE_STATE_RECORDING || current_file == NULL || s == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Rollover check (before we add to batch)
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - (uint32_t)current_file_open_ms) >= FILE_ROLLOVER_MS) {
        // Flush remaining, close, open new file
        ESP_LOGI(TAG, "Rollover: closing %s after ~%" PRIu32 " ms",
                 current_filename, (uint32_t)(now_ms - (uint32_t)current_file_open_ms));

        (void)flush_batch_locked(true);

        fflush(current_file);
        fsync(fileno(current_file));
        fclose(current_file);
        current_file = NULL;

        file_index_in_session++;
        if (open_new_csv_file() != ESP_OK) {
            current_state = STORAGE_STATE_ERROR;
            return ESP_FAIL;
        }
        s_last_flush_ms = now_ms;
    }

    // Add to batch
    s_batch[s_batch_len++] = *s;
    current_file_samples++;

    // Flush when full or time window elapsed
    if (s_batch_len >= BATCH_CAP || (now_ms - s_last_flush_ms) >= FLUSH_INTERVAL_MS) {
        esp_err_t err = flush_batch_locked(false);
        if (err != ESP_OK) {
            current_state = STORAGE_STATE_ERROR;
            return err;
        }
        s_last_flush_ms = now_ms;
    }

    return ESP_OK;
}

// ---------------- Back-compat shim (your old API still works) ---------------

esp_err_t storage_log_imu_sample(uint32_t timestamp, float ax, float ay, float az,
                                 float gx, float gy, float gz, float temp)
{
    bno055_sample_t s = {0};
    s.t_ms = timestamp;

    // Fill what we have; leave others 0 or NAN if you prefer
    s.ax = ax; s.ay = ay; s.az = az;

    // You labeled gyro as rad/s historically; your struct says deg/s.
    // If your upstream gives rad/s, convert to deg/s:
    // s.gx = gx * 57.2957795f; s.gy = gy * 57.2957795f; s.gz = gz * 57.2957795f;
    // If it's already deg/s, assign directly:
    s.gx = gx; s.gy = gy; s.gz = gz;

    // Mag unknown -> NAN helps downstream parsers notice "missing"
    s.mx = NAN; s.my = NAN; s.mz = NAN;

    // Euler/Quat unknown:
    s.roll = NAN; s.pitch = NAN; s.yaw = NAN;
    s.qw = NAN; s.qx = NAN; s.qy = NAN; s.qz = NAN;

    s.temp = temp;

    // Calib unknown
    s.sys_cal = 0; s.gyro_cal = 0; s.accel_cal = 0; s.mag_cal = 0;

    return storage_enqueue_bno_sample(&s);
}

/**
 * @brief Copy the current filename into the provided buffer.
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

esp_err_t storage_get_current_file_size(uint32_t* size) {
    if (size == NULL) return ESP_ERR_INVALID_ARG;
    *size = current_file_size;  // maintained on flush
    return ESP_OK;
}

esp_err_t storage_get_current_sample_count(uint32_t* count) {
    if (count == NULL) return ESP_ERR_INVALID_ARG;
    *count = current_file_samples;
    return ESP_OK;
}

// ---------------- Delete/list helpers (now removes CSV and BIN) --------------

esp_err_t storage_delete_all_files(void) {
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Deleting all CSV/BIN files...");

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
        bool is_reg = true; // Some libcs don't set d_type
#endif
        if (!is_reg) continue;

        const char* name = entry->d_name;
        bool is_csv = (strstr(name, ".csv") || strstr(name, ".CSV"));
        bool is_bin = (strstr(name, ".bin") || strstr(name, ".BIN"));
        if (!is_csv && !is_bin) continue;

        char full_path[320];
        int n = snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, name);
        if (n < 0 || (size_t)n >= sizeof(full_path)) {
            ESP_LOGW(TAG, "Path truncated, skipping delete");
            continue;
        }

        if (unlink(full_path) == 0) {
            deleted_count++;
            ESP_LOGI(TAG, "Deleted: %s", name);
        } else {
            ESP_LOGW(TAG, "Failed to delete: %s", name);
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "Deleted %" PRIu32 " files", deleted_count);
    return ESP_OK;
}

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
        if (!is_reg) continue;

        if (strstr(entry->d_name, ".csv") || strstr(entry->d_name, ".CSV") ||
            strstr(entry->d_name, ".bin") || strstr(entry->d_name, ".BIN")) {
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

// --------- CSV header now matches bno055_sample_t -----------
static esp_err_t create_csv_header(FILE* file) {
    if (file == NULL) return ESP_ERR_INVALID_ARG;

    int ret = fprintf(file,
        "timestamp_ms,"
        "ax_mps2,ay_mps2,az_mps2,"
        "gx_dps,gy_dps,gz_dps,"
        "mx_uT,my_uT,mz_uT,"
        "roll_deg,pitch_deg,yaw_deg,"
        "qw,qx,qy,qz,"
        "temp_c,"
        "sys_cal,gyro_cal,accel_cal,mag_cal\n");
    if (ret < 0) return ESP_FAIL;

    fflush(file);
    fsync(fileno(file));
    return ESP_OK;
}

/**
 * @brief Open a new data file using a short 8.3 name, write header if CSV, update bookkeeping.
 * (Kept function name for minimal changes to your code.)
 */
static esp_err_t open_new_csv_file(void) {
    // Close any existing file first (safety)
    if (current_file) {
        fflush(current_file);
        fsync(fileno(current_file));
        fclose(current_file);
        current_file = NULL;
    }

    // Name pattern: S<5-digit time><2-digit index>.{CSV|BIN}  -> always 8.3
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t t5 = now_ms % 100000;            // last 5 digits
    uint32_t idx2 = file_index_in_session % 100;

    char filename[13];  // "S12345A7.CSV"/"S12345A7.BIN"
    if (STORAGE_TO_BINARY) {
        snprintf(filename, sizeof(filename), "S%05" PRIu32 "%02" PRIu32 ".BIN", t5, idx2);
    } else {
        snprintf(filename, sizeof(filename), "S%05" PRIu32 "%02" PRIu32 ".CSV", t5, idx2);
    }

    char full_path[96];
    int n = snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(full_path)) {
        ESP_LOGE(TAG, "Full path too long");
        return ESP_FAIL;
    }

    current_file = fopen(full_path, "wb");  // works for CSV or BIN
    if (!current_file) {
        ESP_LOGE(TAG, "Failed to create file: %s (errno=%d, %s)",
                full_path, errno, strerror(errno));
        return ESP_FAIL;
    }

    // Add a stdio buffer to reduce syscalls
    static char filebuf[4096];
    setvbuf(current_file, filebuf, _IOFBF, sizeof(filebuf));

    if (!STORAGE_TO_BINARY) {
        // CSV header
        esp_err_t ret = create_csv_header(current_file);
        if (ret != ESP_OK) {
            fclose(current_file);
            current_file = NULL;
            return ret;
        }
    }

    // Bookkeeping
    snprintf(current_filename, sizeof(current_filename), "%s", filename);
    current_file_samples = 0;
    current_file_size = 0;
    current_file_open_ms = now_ms;

    ESP_LOGI(TAG, "Opened new %s: %s", STORAGE_TO_BINARY ? "BIN" : "CSV", current_filename);
    return ESP_OK;
}

// ---------------- batch flush implementation -----------------

static esp_err_t flush_batch_locked(bool force)
{
    if (!current_file || s_batch_len == 0) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;

    if (STORAGE_TO_BINARY) {
        size_t n = fwrite(s_batch, sizeof(bno055_sample_t), s_batch_len, current_file);
        if (n != s_batch_len) {
            ESP_LOGE(TAG, "BIN flush failed (wrote %u of %u)", (unsigned)n, (unsigned)s_batch_len);
            err = ESP_FAIL;
        } else {
            current_file_size += (uint32_t)(n * sizeof(bno055_sample_t));
        }
    } else {
        // CSV: write one line per struct, sum bytes for size
        uint32_t bytes_total = 0;
        for (size_t i = 0; i < s_batch_len; ++i) {
            bno055_sample_t *s = &s_batch[i];
            int w = fprintf(current_file,
                "%" PRIu32 ","
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,"
                "%.7f,%.7f,%.7f,%.7f,"
                "%.2f,"
                "%u,%u,%u,%u\n",
                s->t_ms,
                s->ax, s->ay, s->az,
                s->gx, s->gy, s->gz,
                s->mx, s->my, s->mz,
                s->roll, s->pitch, s->yaw,
                s->qw, s->qx, s->qy, s->qz,
                s->temp,
                (unsigned)s->sys_cal, (unsigned)s->gyro_cal, (unsigned)s->accel_cal, (unsigned)s->mag_cal
            );
            if (w < 0) { err = ESP_FAIL; break; }
            bytes_total += (uint32_t)w;
        }
        current_file_size += bytes_total;
    }

    if (err == ESP_OK) {
        fflush(current_file);
        fsync(fileno(current_file));
        s_batch_len = 0;
    }
    return err;
}

// =======================================================
// Added helpers: listing to text + BLE-friendly streaming
// (unchanged from your file except minor comment)
// =======================================================

static __attribute__((unused)) esp_err_t list_files_to_text(char *out, size_t out_cap, uint32_t *count_out) {
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

        const char* name = entry->d_name;
        if (!(strstr(name, ".csv") || strstr(name, ".CSV") ||
              strstr(name, ".bin") || strstr(name, ".BIN"))) continue;

        size_t name_len = strnlen(name, 255);
        if (used + name_len + 1 >= out_cap) break;  // +1 for '\n'

        memcpy(out + used, name, name_len);
        used += name_len;
        out[used++] = '\n';
        count++;
    }
    closedir(dir);

    if (used < out_cap) out[used] = '\0';
    *count_out = count;
    return ESP_OK;
}

// ... (ble_chunk_and_send, storage_stream_file_over_ble, storage_read_file_into_buffer)
//     keep exactly as in your file above (no changes needed)
