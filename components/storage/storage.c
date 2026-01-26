#include "storage.h"
#include "psram_buffer.h"
#include "protobuf_utils.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <dirent.h>
#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>

static const char *TAG = "STORAGE";

// ============== Configuration ==============
// FLUSH_INTERVAL_MS: How often the background task checks for data to flush to SD card
// This is the batch write frequency - keeps writes efficient by batching multiple samples
#define FLUSH_INTERVAL_MS       500

// FLUSH_BATCH_SIZE: Number of samples to write in each flush operation
#define FLUSH_BATCH_SIZE        256

// FILE_ROLLOVER_MS: Create a new file after this duration for manageability
// Separate from flush interval - this controls file rotation, not write frequency
#define FILE_ROLLOVER_MS        (60 * 1000)

// SD_FULL_THRESHOLD_BYTES: Stop recording when less than this space remains
#define SD_FULL_THRESHOLD_BYTES (1 * 1024 * 1024)  // 1MB

// ============== Storage state ==============
static bool storage_initialized = false;
static bool storage_mounted = false;
static storage_state_t current_state = STORAGE_STATE_IDLE;

static FILE *current_file = NULL;
static char current_filename[STORAGE_MAX_FILENAME_LEN] = {0};
static uint32_t current_file_samples = 0;
static uint32_t current_file_size = 0;
static uint32_t current_session_start = 0;
static uint32_t total_session_samples = 0;

// File rollover state
static uint64_t current_file_open_ms = 0;
static uint32_t file_index_in_session = 0;

// Mount point and card
static const char mount_point[] = "/sdcard";
static sdmmc_card_t *card = NULL;

// ============== Background flush task ==============
static TaskHandle_t flush_task_handle = NULL;
static volatile bool flush_task_stop = false;
static SemaphoreHandle_t file_mutex = NULL;

// ============== Recent samples ring (small, in internal RAM) ==============
#define RECENT_RING_CAP 256
static bno055_sample_t s_recent_ring[RECENT_RING_CAP];
static size_t s_recent_head = 0;
static size_t s_recent_count = 0;

static inline void recent_ring_push(const bno055_sample_t *s) {
    s_recent_ring[s_recent_head] = *s;
    s_recent_head = (s_recent_head + 1) % RECENT_RING_CAP;
    if (s_recent_count < RECENT_RING_CAP) s_recent_count++;
}

// ============== Internal function prototypes ==============
static esp_err_t mount_sd_card(void);
static esp_err_t unmount_sd_card(void);
static esp_err_t open_new_data_file(void);
static esp_err_t close_current_file(void);
static esp_err_t flush_psram_to_sd(void);
static void flush_task(void *arg);
static bool check_sd_space(void);

// ============== Public API Implementation ==============

esp_err_t storage_init(void) {
    if (storage_initialized) {
        ESP_LOGW(TAG, "Storage already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing storage system...");

    // Create file access mutex
    file_mutex = xSemaphoreCreateMutex();
    if (!file_mutex) {
        ESP_LOGE(TAG, "Failed to create file mutex");
        return ESP_ERR_NO_MEM;
    }

    // Mount SD card
    esp_err_t ret = mount_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        vSemaphoreDelete(file_mutex);
        file_mutex = NULL;
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }
    storage_mounted = true;

    // Initialize PSRAM ring buffer
    ret = psram_buffer_init(0);  // Use default capacity
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PSRAM buffer: %s", esp_err_to_name(ret));
        unmount_sd_card();
        vSemaphoreDelete(file_mutex);
        file_mutex = NULL;
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }

    storage_initialized = true;
    current_state = STORAGE_STATE_IDLE;

    ESP_LOGI(TAG, "Storage system initialized successfully");
    ESP_LOGI(TAG, "  PSRAM buffer capacity: %zu samples", psram_buffer_get_capacity());
    
    return ESP_OK;
}

esp_err_t storage_deinit(void) {
    if (!storage_initialized) {
        return ESP_OK;
    }

    // Stop recording if active
    if (current_state == STORAGE_STATE_RECORDING) {
        storage_stop_session();
    }

    // Deinitialize PSRAM buffer
    psram_buffer_deinit();

    // Unmount SD card
    esp_err_t ret = unmount_sd_card();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
    }

    if (file_mutex) {
        vSemaphoreDelete(file_mutex);
        file_mutex = NULL;
    }

    storage_mounted = false;
    storage_initialized = false;
    current_state = STORAGE_STATE_IDLE;

    ESP_LOGI(TAG, "Storage system deinitialized");
    return ret;
}

bool storage_is_available(void) {
    return storage_initialized && storage_mounted && 
           (current_state != STORAGE_STATE_ERROR);
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

    ESP_LOGI(TAG, "Starting recording session...");

    // Close any stale file
    if (current_file) {
        close_current_file();
    }

    // Wipe old data files at session start
    ESP_LOGI(TAG, "Clearing previous session data...");
    esp_err_t wipe_ret = storage_delete_all_files();
    if (wipe_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear old files (continuing anyway)");
    }

    // Clear PSRAM buffer
    psram_buffer_clear();

    // Reset session counters
    file_index_in_session = 0;
    total_session_samples = 0;
    current_session_start = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // Open first data file
    esp_err_t ret = open_new_data_file();
    if (ret != ESP_OK) {
        current_state = STORAGE_STATE_ERROR;
        return ret;
    }

    // Start background flush task
    // Increased stack size to 8192 to prevent overflow (protobuf encoding needs more stack)
    flush_task_stop = false;
    BaseType_t task_ret = xTaskCreate(flush_task, "storage_flush", 8192, NULL, 5, &flush_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create flush task");
        close_current_file();
        current_state = STORAGE_STATE_ERROR;
        return ESP_FAIL;
    }

    current_state = STORAGE_STATE_RECORDING;
    ESP_LOGI(TAG, "Recording session started (file: %s)", current_filename);
    return ESP_OK;
}

esp_err_t storage_stop_session(void) {
    if (current_state != STORAGE_STATE_RECORDING && current_state != STORAGE_STATE_SD_FULL) {
        ESP_LOGW(TAG, "Not currently recording");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping recording session...");

    // Stop flush task
    if (flush_task_handle) {
        flush_task_stop = true;
        // Wait for task to finish (max 2 seconds)
        for (int i = 0; i < 40 && flush_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (flush_task_handle != NULL) {
            ESP_LOGW(TAG, "Flush task did not terminate gracefully");
            vTaskDelete(flush_task_handle);
            flush_task_handle = NULL;
        }
    }

    // Final flush of remaining PSRAM data
    if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        flush_psram_to_sd();  // Flush any remaining samples
        close_current_file();
        xSemaphoreGive(file_mutex);
    }

    uint32_t session_duration = (uint32_t)(esp_timer_get_time() / 1000ULL) - current_session_start;
    ESP_LOGI(TAG, "Session stopped:");
    ESP_LOGI(TAG, "  Duration: %" PRIu32 " ms", session_duration);
    ESP_LOGI(TAG, "  Total samples: %" PRIu32, total_session_samples);
    ESP_LOGI(TAG, "  Files created: %" PRIu32, file_index_in_session + 1);

    // Get buffer stats
    psram_buffer_stats_t stats;
    if (psram_buffer_get_stats(&stats) == ESP_OK && stats.overflows > 0) {
        ESP_LOGW(TAG, "  Samples dropped (overflow): %zu", stats.overflows);
    }

    current_state = STORAGE_STATE_IDLE;
    return ESP_OK;
}

bool storage_is_recording(void) {
    return current_state == STORAGE_STATE_RECORDING;
}

esp_err_t storage_push_sample(const bno055_sample_t *sample) {
    if (!sample) {
        return ESP_ERR_INVALID_ARG;
    }
    if (current_state != STORAGE_STATE_RECORDING) {
        return ESP_ERR_INVALID_STATE;
    }

    // Push to PSRAM ring buffer (fast, non-blocking)
    esp_err_t ret = psram_buffer_push(sample);
    if (ret != ESP_OK) {
        return ret;
    }

    // Also push to small in-RAM recent ring for optional inspection
    recent_ring_push(sample);

    return ESP_OK;
}

esp_err_t storage_enqueue_bno_sample(const bno055_sample_t *sample) {
    return storage_push_sample(sample);
}

esp_err_t storage_log_imu_sample(uint32_t timestamp, float ax, float ay, float az,
                                  float gx, float gy, float gz, float temp) {
    bno055_sample_t s = {0};
    s.t_ms = timestamp;
    s.ax = ax; s.ay = ay; s.az = az;
    s.gx = gx; s.gy = gy; s.gz = gz;
    s.mx = NAN; s.my = NAN; s.mz = NAN;
    s.roll = NAN; s.pitch = NAN; s.yaw = NAN;
    s.qw = NAN; s.qx = NAN; s.qy = NAN; s.qz = NAN;
    s.temp = temp;
    s.sys_cal = 0; s.gyro_cal = 0; s.accel_cal = 0; s.mag_cal = 0;
    return storage_push_sample(&s);
}

esp_err_t storage_push_bno_sample(const bno055_sample_t *s) {
    return storage_push_sample(s);
}

size_t storage_copy_recent(bno055_sample_t *out, size_t max_out) {
    if (!out || max_out == 0) return 0;

    size_t n = (s_recent_count < max_out) ? s_recent_count : max_out;
    for (size_t i = 0; i < n; ++i) {
        size_t idx = (s_recent_head + RECENT_RING_CAP - n + i) % RECENT_RING_CAP;
        out[i] = s_recent_ring[idx];
    }
    return n;
}

esp_err_t storage_get_current_filename(char *filename, size_t max_len) {
    if (!filename || max_len == 0) {
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

esp_err_t storage_get_current_file_size(uint32_t *size) {
    if (!size) return ESP_ERR_INVALID_ARG;
    *size = current_file_size;
    return ESP_OK;
}

esp_err_t storage_get_current_sample_count(uint32_t *count) {
    if (!count) return ESP_ERR_INVALID_ARG;
    *count = total_session_samples;
    return ESP_OK;
}

esp_err_t storage_get_free_space(uint64_t *free_bytes) {
    if (!free_bytes) return ESP_ERR_INVALID_ARG;
    
    struct statvfs stat;
    int ret = statvfs(mount_point, &stat);
    if (ret != 0) {
        // statvfs may not be implemented on all filesystems (errno 88 = ENOSYS)
        // This is expected for some SD card filesystems, so use DEBUG level instead of ERROR
        ESP_LOGD(TAG, "statvfs not available: errno=%d (%s) - space check disabled", errno, strerror(errno));
        return ESP_FAIL;
    }
    
    *free_bytes = (uint64_t)stat.f_bsize * stat.f_bavail;
    ESP_LOGD(TAG, "SD free space: %llu bytes (block size: %lu, available blocks: %lu)",
             (unsigned long long)*free_bytes, (unsigned long)stat.f_bsize, (unsigned long)stat.f_bavail);
    return ESP_OK;
}

esp_err_t storage_get_stats(storage_stats_t *stats) {
    if (!stats) return ESP_ERR_INVALID_ARG;

    memset(stats, 0, sizeof(*stats));
    
    stats->psram_buffer_count = psram_buffer_get_count();
    stats->psram_buffer_capacity = psram_buffer_get_capacity();
    stats->total_samples_written = total_session_samples;
    stats->current_file_size = current_file_size;
    stats->is_recording = (current_state == STORAGE_STATE_RECORDING);
    stats->is_sd_full = (current_state == STORAGE_STATE_SD_FULL);

    psram_buffer_stats_t buf_stats;
    if (psram_buffer_get_stats(&buf_stats) == ESP_OK) {
        stats->total_samples_dropped = buf_stats.overflows;
    }

    storage_get_free_space(&stats->sd_free_bytes);

    return ESP_OK;
}

// ============== File management ==============

esp_err_t storage_delete_all_files(void) {
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Deleting all data files...");

    if (current_state == STORAGE_STATE_RECORDING) {
        storage_stop_session();
    }

    DIR *dir = opendir(mount_point);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory %s", mount_point);
        return ESP_FAIL;
    }

    struct dirent *entry;
    uint32_t deleted_count = 0;

    while ((entry = readdir(dir)) != NULL) {
#if defined(DT_REG)
        bool is_reg = (entry->d_type == DT_REG);
#else
        bool is_reg = true;
#endif
        if (!is_reg) continue;

        const char *name = entry->d_name;
        bool is_csv = (strstr(name, ".csv") || strstr(name, ".CSV"));
        bool is_bin = (strstr(name, ".bin") || strstr(name, ".BIN"));
        if (!is_csv && !is_bin) continue;

        char full_path[320];
        int n = snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, name);
        if (n < 0 || (size_t)n >= sizeof(full_path)) continue;

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

esp_err_t storage_list_files(char files[][32], uint32_t max_files, uint32_t *actual_count) {
    if (!storage_is_available() || !files || !actual_count) {
        return ESP_ERR_INVALID_ARG;
    }

    DIR *dir = opendir(mount_point);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory");
        return ESP_FAIL;
    }

    struct dirent *entry;
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

static int name_cmp_asc(const void *a, const void *b) {
    const char *sa = (const char*)a;
    const char *sb = (const char*)b;
    return strncmp(sa, sb, 32);
}

esp_err_t storage_list_bin_files(char files[][32], uint32_t max_files, 
                                  uint32_t *actual_count, bool sort_asc) {
    if (!storage_is_available() || !files || !actual_count) {
        return ESP_ERR_INVALID_ARG;
    }

    DIR *dir = opendir(mount_point);
    if (!dir) return ESP_FAIL;

    uint32_t count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < max_files) {
#if defined(DT_REG)
        bool is_reg = (entry->d_type == DT_REG);
#else
        bool is_reg = true;
#endif
        if (!is_reg) continue;
        const char *n = entry->d_name;
        if (!(strstr(n, ".BIN") || strstr(n, ".bin"))) continue;

        strncpy(files[count], n, 31);
        files[count][31] = '\0';
        count++;
    }
    closedir(dir);

    if (sort_asc && count > 1) {
        qsort(files, count, 32, name_cmp_asc);
    }
    *actual_count = count;
    return ESP_OK;
}

// ============== Data reading ==============

esp_err_t storage_read_bin_file_into_buffer(const char *filename,
                                            bno055_sample_t *buf,
                                            size_t cap,
                                            size_t *out_len) {
    if (!filename || !buf || !out_len) return ESP_ERR_INVALID_ARG;
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    char path[160];
    int n = snprintf(path, sizeof(path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(path)) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(path, "rb");
    if (!f) return ESP_FAIL;

    size_t total_read = 0;

    // Read length-delimited protobuf batches
    while (total_read < cap) {
        size_t decoded = 0;
        esp_err_t ret = protobuf_read_delimited(f, buf + total_read, cap - total_read, &decoded);
        if (ret == ESP_ERR_NOT_FOUND) {
            break;  // EOF
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Error reading protobuf batch: %s", esp_err_to_name(ret));
            break;
        }
        total_read += decoded;
    }

    fclose(f);
    *out_len = total_read;
    return ESP_OK;
}

esp_err_t storage_iterate_bin_files(storage_samples_cb_t cb, void *ctx, size_t chunk) {
    if (!cb || chunk == 0) return ESP_ERR_INVALID_ARG;
    if (!storage_is_available()) return ESP_ERR_INVALID_STATE;

    char names[64][32];
    uint32_t count = 0;
    esp_err_t err = storage_list_bin_files(names, 64, &count, true);
    if (err != ESP_OK) return err;

    if (count == 0) return ESP_OK;

    bno055_sample_t *buf = malloc(chunk * sizeof(bno055_sample_t));
    if (!buf) return ESP_ERR_NO_MEM;

    for (uint32_t i = 0; i < count; ++i) {
        size_t got = 0;
        err = storage_read_bin_file_into_buffer(names[i], buf, chunk, &got);
        if (err == ESP_OK && got > 0) {
            cb(buf, got, ctx);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buf);
    return ESP_OK;
}

static int cmp_names_asc(const void *a, const void *b) {
    const char * const *sa = (const char * const *)a;
    const char * const *sb = (const char * const *)b;
    return strcmp(*sa, *sb);
}

esp_err_t storage_read_all_bin_into_buffer(bno055_sample_t *out,
                                           size_t out_cap,
                                           size_t *out_len) {
    if (!out || !out_len) return ESP_ERR_INVALID_ARG;
    *out_len = 0;

    if (!storage_is_available()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Collect .BIN file names
    DIR *dir = opendir(mount_point);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed", mount_point);
        return ESP_FAIL;
    }

    size_t names_cap = 16, names_len = 0;
    char **names = malloc(names_cap * sizeof(char *));
    if (!names) {
        closedir(dir);
        return ESP_ERR_NO_MEM;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
#if defined(DT_REG)
        if (entry->d_type != DT_REG) continue;
#endif
        const char *n = entry->d_name;
        bool is_bin = (strstr(n, ".BIN") != NULL) || (strstr(n, ".bin") != NULL);
        if (!is_bin) continue;

        size_t L = strnlen(n, 255);
        if (L == 0 || L >= 64) continue;

        if (names_len == names_cap) {
            size_t new_cap = names_cap * 2;
            char **tmp = realloc(names, new_cap * sizeof(char *));
            if (!tmp) break;
            names = tmp;
            names_cap = new_cap;
        }

        names[names_len] = malloc(L + 1);
        if (!names[names_len]) break;
        memcpy(names[names_len], n, L);
        names[names_len][L] = '\0';
        names_len++;
    }
    closedir(dir);

    if (names_len == 0) {
        free(names);
        ESP_LOGW(TAG, "No .BIN files found on SD");
        return ESP_OK;
    }

    // Sort ascending by filename
    qsort(names, names_len, sizeof(char *), cmp_names_asc);

    // Read files sequentially
    size_t written = 0;
    esp_err_t ret = ESP_OK;

    for (size_t fi = 0; fi < names_len; ++fi) {
        size_t got = 0;
        size_t remaining = out_cap - written;
        if (remaining == 0) {
            ret = ESP_ERR_NO_MEM;
            break;
        }

        esp_err_t read_ret = storage_read_bin_file_into_buffer(names[fi], out + written, remaining, &got);
        if (read_ret == ESP_OK) {
            written += got;
            ESP_LOGI(TAG, "Read %zu samples from %s", got, names[fi]);
        } else {
            ESP_LOGW(TAG, "Failed to read %s: %s", names[fi], esp_err_to_name(read_ret));
        }
    }

    for (size_t i = 0; i < names_len; ++i) free(names[i]);
    free(names);

    *out_len = written;
    ESP_LOGI(TAG, "Total samples read: %zu", written);
    return ret;
}

// ============== Internal helper functions ==============

static esp_err_t mount_sd_card(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_FORMSYNC_SD_MOSI_GPIO,
        .miso_io_num = CONFIG_FORMSYNC_SD_MISO_GPIO,
        .sclk_io_num = CONFIG_FORMSYNC_SD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ESP_LOGI(TAG, "SD SPI pins: MOSI=%d, MISO=%d, SCK=%d, CS=%d",
             CONFIG_FORMSYNC_SD_MOSI_GPIO, CONFIG_FORMSYNC_SD_MISO_GPIO,
             CONFIG_FORMSYNC_SD_SCK_GPIO, CONFIG_FORMSYNC_SD_CS_GPIO);

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_FORMSYNC_SD_CS_GPIO;
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
        spi_bus_free(SPI2_HOST);
        return ret;
    }
    return ESP_OK;
}

static esp_err_t open_new_data_file(void) {
    // Close any existing file
    if (current_file) {
        close_current_file();
    }

    // Generate filename: S<5-digit time><2-digit index>.BIN
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t t5 = now_ms % 100000;
    uint32_t idx2 = file_index_in_session % 100;

    char filename[13];
    snprintf(filename, sizeof(filename), "S%05" PRIu32 "%02" PRIu32 ".BIN", t5, idx2);

    char full_path[96];
    int n = snprintf(full_path, sizeof(full_path), "%s/%s", mount_point, filename);
    if (n < 0 || (size_t)n >= sizeof(full_path)) {
        ESP_LOGE(TAG, "Full path too long");
        return ESP_FAIL;
    }

    current_file = fopen(full_path, "wb");
    if (!current_file) {
        ESP_LOGE(TAG, "Failed to create file: %s (errno=%d, %s)",
                 full_path, errno, strerror(errno));
        return ESP_FAIL;
    }

    // Set up buffering
    static char filebuf[4096];
    setvbuf(current_file, filebuf, _IOFBF, sizeof(filebuf));

    // Update bookkeeping
    snprintf(current_filename, sizeof(current_filename), "%s", filename);
    current_file_samples = 0;
    current_file_size = 0;
    current_file_open_ms = now_ms;

    ESP_LOGI(TAG, "Opened new data file: %s (protobuf format)", current_filename);
    return ESP_OK;
}

static esp_err_t close_current_file(void) {
    if (!current_file) {
        return ESP_OK;
    }

    fflush(current_file);
    fsync(fileno(current_file));
    fclose(current_file);
    current_file = NULL;

    ESP_LOGI(TAG, "Closed file: %s (samples: %" PRIu32 ", size: %" PRIu32 " bytes)",
             current_filename, current_file_samples, current_file_size);

    memset(current_filename, 0, sizeof(current_filename));
    return ESP_OK;
}

static bool check_sd_space(void) {
    uint64_t free_bytes = 0;
    esp_err_t ret = storage_get_free_space(&free_bytes);
    if (ret != ESP_OK) {
        // statvfs may not be implemented on this filesystem (errno 88 = ENOSYS)
        // This is expected and non-critical - assume space is available
        // Only log at DEBUG level to reduce spam (already logged in storage_get_free_space)
        return true;  // Assume space is available if check fails
    }
    
    if (free_bytes < SD_FULL_THRESHOLD_BYTES) {
        ESP_LOGE(TAG, "SD card low on space: %llu bytes free (threshold: %d bytes)", 
                 (unsigned long long)free_bytes, SD_FULL_THRESHOLD_BYTES);
        return false;
    }
    
    return true;
}

static esp_err_t flush_psram_to_sd(void) {
    if (!current_file) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check SD space before write
    if (!check_sd_space()) {
        ESP_LOGW(TAG, "SD card full - stopping recording");
        current_state = STORAGE_STATE_SD_FULL;
        return ESP_ERR_NO_MEM;
    }

    // Check for file rollover
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - (uint32_t)current_file_open_ms) >= FILE_ROLLOVER_MS) {
        ESP_LOGI(TAG, "File rollover: closing %s after ~%" PRIu32 " ms",
                 current_filename, (uint32_t)(now_ms - (uint32_t)current_file_open_ms));
        
        close_current_file();
        file_index_in_session++;
        if (open_new_data_file() != ESP_OK) {
            return ESP_FAIL;
        }
    }

    // Pop samples from PSRAM buffer
    bno055_sample_t *batch = malloc(FLUSH_BATCH_SIZE * sizeof(bno055_sample_t));
    if (!batch) {
        ESP_LOGE(TAG, "Failed to allocate flush batch buffer");
        return ESP_ERR_NO_MEM;
    }

    size_t popped = psram_buffer_pop_batch(batch, FLUSH_BATCH_SIZE);
    if (popped == 0) {
        free(batch);
        return ESP_OK;  // Nothing to flush
    }

    // Write as length-delimited protobuf
    esp_err_t ret = protobuf_write_delimited(current_file, batch, popped);
    free(batch);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write protobuf batch: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update stats
    total_session_samples += popped;
    current_file_samples += popped;
    
    // Estimate file size (actual size tracked by seeking would be expensive)
    current_file_size += popped * 110;  // ~110 bytes per sample in protobuf

    // Periodic progress logging (every 1000 samples)
    if (total_session_samples % 1000 < popped) {
        ESP_LOGI(TAG, "Flushed: total=%" PRIu32 " samples, buffer=%zu",
                 total_session_samples, psram_buffer_get_count());
    }

    return ESP_OK;
}

static void flush_task(void *arg) {
    ESP_LOGI(TAG, "Flush task started (interval: %d ms)", FLUSH_INTERVAL_MS);

    while (!flush_task_stop) {
        // Wait for flush interval or watermark trigger
        vTaskDelay(pdMS_TO_TICKS(FLUSH_INTERVAL_MS));

        if (flush_task_stop) break;

        // Check if we should flush (watermark or periodic)
        bool should_flush = psram_buffer_is_above_watermark() || 
                            (psram_buffer_get_count() > 0);

        if (should_flush && current_state == STORAGE_STATE_RECORDING) {
            if (xSemaphoreTake(file_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Flush until buffer is below watermark (limit iterations to prevent watchdog)
                int flush_iterations = 0;
                const int MAX_FLUSH_ITERATIONS = 10;  // Prevent infinite loop
                
                while (psram_buffer_get_count() > 0 && 
                       current_state == STORAGE_STATE_RECORDING &&
                       flush_iterations < MAX_FLUSH_ITERATIONS) {
                    esp_err_t ret = flush_psram_to_sd();
                    flush_iterations++;
                    
                    if (ret != ESP_OK) {
                        if (ret == ESP_ERR_NO_MEM) {
                            // SD full - stop recording
                            break;
                        }
                        ESP_LOGW(TAG, "Flush error: %s", esp_err_to_name(ret));
                        break;
                    }
                    
                    // Yield between batches to prevent watchdog
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                if (flush_iterations >= MAX_FLUSH_ITERATIONS && psram_buffer_get_count() > 0) {
                    ESP_LOGW(TAG, "Flush iteration limit reached, will continue next cycle");
                }
                
                xSemaphoreGive(file_mutex);
            }
        }
    }

    ESP_LOGI(TAG, "Flush task stopped");
    flush_task_handle = NULL;
    vTaskDelete(NULL);
}
