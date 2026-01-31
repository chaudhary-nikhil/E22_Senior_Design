#include "psram_buffer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "PSRAM_BUFFER";

// Ring buffer structure
typedef struct {
    bno055_sample_t *buffer;      // Sample storage array
    size_t capacity;              // Maximum samples
    size_t head;                  // Write position (next write goes here)
    size_t tail;                  // Read position (next read comes from here)
    size_t count;                 // Current number of samples
    
    // Statistics
    size_t total_pushed;
    size_t total_popped;
    size_t overflows;
    
    // Watermark
    size_t watermark_threshold;
    
    // Thread safety
    SemaphoreHandle_t mutex;
    
    bool initialized;
    bool is_psram;                // True if allocated from PSRAM
} ring_buffer_t;

static ring_buffer_t s_ring = {0};

// Helper to advance index with wrap-around
static inline size_t advance_index(size_t index, size_t capacity) {
    return (index + 1) % capacity;
}

esp_err_t psram_buffer_init(size_t capacity_samples)
{
    if (s_ring.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Use default if not specified
    if (capacity_samples == 0) {
        capacity_samples = CONFIG_PSRAM_BUFFER_CAPACITY_SAMPLES;
    }

    size_t buffer_size = capacity_samples * sizeof(bno055_sample_t);
    ESP_LOGI(TAG, "Initializing PSRAM ring buffer: %zu samples (%zu KB)", 
             capacity_samples, buffer_size / 1024);

    // Try to allocate from PSRAM first
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "Free PSRAM: %zu bytes", free_psram);

    s_ring.buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    s_ring.is_psram = true;
    
    if (!s_ring.buffer) {
        ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM...");
        // Fall back to smaller internal RAM buffer
        capacity_samples = 1024;  // ~90KB
        buffer_size = capacity_samples * sizeof(bno055_sample_t);
        s_ring.buffer = malloc(buffer_size);
        s_ring.is_psram = false;
        
        if (!s_ring.buffer) {
            ESP_LOGE(TAG, "Failed to allocate buffer");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGW(TAG, "Using internal RAM buffer (reduced capacity: %zu samples)", capacity_samples);
    } else {
        ESP_LOGI(TAG, "Allocated %zu KB from PSRAM", buffer_size / 1024);
    }

    // Create mutex
    s_ring.mutex = xSemaphoreCreateMutex();
    if (!s_ring.mutex) {
        if (s_ring.is_psram) {
            heap_caps_free(s_ring.buffer);
        } else {
            free(s_ring.buffer);
        }
        s_ring.buffer = NULL;
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize state
    s_ring.capacity = capacity_samples;
    s_ring.head = 0;
    s_ring.tail = 0;
    s_ring.count = 0;
    s_ring.total_pushed = 0;
    s_ring.total_popped = 0;
    s_ring.overflows = 0;
    s_ring.watermark_threshold = (capacity_samples * CONFIG_PSRAM_BUFFER_WATERMARK_PERCENT) / 100;
    s_ring.initialized = true;

    ESP_LOGI(TAG, "Ring buffer initialized: capacity=%zu, watermark=%zu (%d%%)",
             s_ring.capacity, s_ring.watermark_threshold, CONFIG_PSRAM_BUFFER_WATERMARK_PERCENT);

    return ESP_OK;
}

void psram_buffer_deinit(void)
{
    if (!s_ring.initialized) {
        return;
    }

    if (s_ring.mutex) {
        xSemaphoreTake(s_ring.mutex, portMAX_DELAY);
    }

    if (s_ring.buffer) {
        if (s_ring.is_psram) {
            heap_caps_free(s_ring.buffer);
        } else {
            free(s_ring.buffer);
        }
        s_ring.buffer = NULL;
    }

    if (s_ring.mutex) {
        SemaphoreHandle_t mutex = s_ring.mutex;
        s_ring.mutex = NULL;
        xSemaphoreGive(mutex);
        vSemaphoreDelete(mutex);
    }

    memset(&s_ring, 0, sizeof(s_ring));
    ESP_LOGI(TAG, "Ring buffer deinitialized");
}

bool psram_buffer_is_initialized(void)
{
    return s_ring.initialized;
}

esp_err_t psram_buffer_push(const bno055_sample_t *sample)
{
    if (!s_ring.initialized || !sample) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Copy sample to buffer at head position
    s_ring.buffer[s_ring.head] = *sample;
    s_ring.head = advance_index(s_ring.head, s_ring.capacity);
    s_ring.total_pushed++;

    if (s_ring.count < s_ring.capacity) {
        // Buffer not full, just increment count
        s_ring.count++;
    } else {
        // Buffer full, advance tail (overwrite oldest)
        s_ring.tail = advance_index(s_ring.tail, s_ring.capacity);
        s_ring.overflows++;
    }

    xSemaphoreGive(s_ring.mutex);
    return ESP_OK;
}

size_t psram_buffer_push_batch(const bno055_sample_t *samples, size_t count)
{
    if (!s_ring.initialized || !samples || count == 0) {
        return 0;
    }

    size_t pushed = 0;
    for (size_t i = 0; i < count; i++) {
        if (psram_buffer_push(&samples[i]) == ESP_OK) {
            pushed++;
        } else {
            break;
        }
    }
    return pushed;
}

size_t psram_buffer_pop_batch(bno055_sample_t *out, size_t max_samples)
{
    if (!s_ring.initialized || !out || max_samples == 0) {
        return 0;
    }

    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    size_t to_pop = (s_ring.count < max_samples) ? s_ring.count : max_samples;
    
    for (size_t i = 0; i < to_pop; i++) {
        out[i] = s_ring.buffer[s_ring.tail];
        s_ring.tail = advance_index(s_ring.tail, s_ring.capacity);
    }
    
    s_ring.count -= to_pop;
    s_ring.total_popped += to_pop;

    xSemaphoreGive(s_ring.mutex);
    return to_pop;
}

size_t psram_buffer_peek(bno055_sample_t *out, size_t max_samples)
{
    if (!s_ring.initialized || !out || max_samples == 0) {
        return 0;
    }

    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }

    size_t to_peek = (s_ring.count < max_samples) ? s_ring.count : max_samples;
    size_t idx = s_ring.tail;
    
    for (size_t i = 0; i < to_peek; i++) {
        out[i] = s_ring.buffer[idx];
        idx = advance_index(idx, s_ring.capacity);
    }

    xSemaphoreGive(s_ring.mutex);
    return to_peek;
}

size_t psram_buffer_get_count(void)
{
    if (!s_ring.initialized) {
        return 0;
    }

    // Use mutex for consistency, but with short timeout
    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        // Return cached value if can't get mutex quickly
        return s_ring.count;
    }

    size_t count = s_ring.count;
    xSemaphoreGive(s_ring.mutex);
    return count;
}

bool psram_buffer_is_above_watermark(void)
{
    if (!s_ring.initialized) {
        return false;
    }
    return s_ring.count >= s_ring.watermark_threshold;
}

bool psram_buffer_is_empty(void)
{
    if (!s_ring.initialized) {
        return true;
    }
    return s_ring.count == 0;
}

bool psram_buffer_is_full(void)
{
    if (!s_ring.initialized) {
        return false;
    }
    return s_ring.count >= s_ring.capacity;
}

esp_err_t psram_buffer_get_stats(psram_buffer_stats_t *stats)
{
    if (!s_ring.initialized || !stats) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    stats->capacity = s_ring.capacity;
    stats->count = s_ring.count;
    stats->total_pushed = s_ring.total_pushed;
    stats->total_popped = s_ring.total_popped;
    stats->overflows = s_ring.overflows;
    stats->watermark_threshold = s_ring.watermark_threshold;
    stats->is_above_watermark = (s_ring.count >= s_ring.watermark_threshold);

    xSemaphoreGive(s_ring.mutex);
    return ESP_OK;
}

void psram_buffer_clear(void)
{
    if (!s_ring.initialized) {
        return;
    }

    if (xSemaphoreTake(s_ring.mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    s_ring.head = 0;
    s_ring.tail = 0;
    s_ring.count = 0;
    // Don't reset statistics - they track lifetime

    xSemaphoreGive(s_ring.mutex);
    ESP_LOGI(TAG, "Buffer cleared");
}

void psram_buffer_set_watermark(uint8_t percent)
{
    if (!s_ring.initialized || percent == 0 || percent >= 100) {
        return;
    }

    if (xSemaphoreTake(s_ring.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    s_ring.watermark_threshold = (s_ring.capacity * percent) / 100;
    ESP_LOGI(TAG, "Watermark set to %u%% (%zu samples)", percent, s_ring.watermark_threshold);

    xSemaphoreGive(s_ring.mutex);
}

size_t psram_buffer_get_capacity(void)
{
    return s_ring.initialized ? s_ring.capacity : 0;
}
