#include "wifi_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <dirent.h>
#include <stdlib.h>

#include "storage.h"
#include "bno055.h"
#include "protobuf_utils.h"

static const char *TAG = "WIFI_SERVER";

// Global state
static wifi_server_state_t current_state = WIFI_SERVER_STATE_IDLE;
static httpd_handle_t server = NULL;
static bool is_syncing = false;
static uint8_t connected_clients = 0;

// Streaming state for SD card -> WiFi transfer
typedef struct {
    char **file_names;           // List of .BIN file names to stream
    size_t file_count;            // Number of files
    size_t current_file_idx;      // Current file being streamed
    FILE *current_file;           // File handle for current file
    size_t total_samples_sent;   // Total samples sent so far
    size_t total_samples_available; // Total samples available (for status)
    bool transfer_complete;       // Flag indicating transfer finished
    SemaphoreHandle_t mutex;      // Mutex to protect streaming state
} streaming_state_t;

static streaming_state_t stream_state = {0};

// Small buffer for reading chunks from SD card (much smaller than full session)
// This buffer is used to accumulate samples from protobuf batches before sending
// Each protobuf batch can contain up to 256 samples, so we need at least that
#define STREAM_CHUNK_SAMPLES 256  // Must be >= max batch size to avoid truncation
#define MAX_BATCH_SAMPLES 256     // Maximum samples per protobuf batch
static bno055_sample_t *stream_chunk_buffer = NULL;

// Initialize streaming state
static esp_err_t init_streaming_state(void) {
    if (stream_state.mutex == NULL) {
        stream_state.mutex = xSemaphoreCreateMutex();
        if (stream_state.mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Allocate small chunk buffer for reading from SD card
    if (stream_chunk_buffer == NULL) {
        stream_chunk_buffer = malloc(STREAM_CHUNK_SAMPLES * sizeof(bno055_sample_t));
        if (stream_chunk_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate stream chunk buffer");
            return ESP_ERR_NO_MEM;
        }
    }
    
    return ESP_OK;
}

// Comparison function for sorting file names
static int cmp_file_names(const void *a, const void *b) {
    const char * const *sa = (const char * const *)a;
    const char * const *sb = (const char * const *)b;
    return strcmp(*sa, *sb);
}

// Cleanup streaming state
static void cleanup_streaming_state(void) {
    if (stream_state.mutex && xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Close current file if open
        if (stream_state.current_file) {
            fclose(stream_state.current_file);
            stream_state.current_file = NULL;
        }
        
        // Free file names list
        if (stream_state.file_names) {
            for (size_t i = 0; i < stream_state.file_count; i++) {
                if (stream_state.file_names[i]) {
                    free(stream_state.file_names[i]);
                }
            }
            free(stream_state.file_names);
            stream_state.file_names = NULL;
        }
        
        stream_state.file_count = 0;
        stream_state.current_file_idx = 0;
        stream_state.total_samples_sent = 0;
        stream_state.total_samples_available = 0;
        stream_state.transfer_complete = false;
        
        xSemaphoreGive(stream_state.mutex);
    } else {
        // If mutex doesn't exist or can't take it, cleanup without lock
        if (stream_state.current_file) {
            fclose(stream_state.current_file);
            stream_state.current_file = NULL;
        }
        
        if (stream_state.file_names) {
            for (size_t i = 0; i < stream_state.file_count; i++) {
                if (stream_state.file_names[i]) {
                    free(stream_state.file_names[i]);
                }
            }
            free(stream_state.file_names);
            stream_state.file_names = NULL;
        }
        
        stream_state.file_count = 0;
        stream_state.current_file_idx = 0;
        stream_state.total_samples_sent = 0;
        stream_state.total_samples_available = 0;
        stream_state.transfer_complete = false;
    }
}

// HTTP handlers
static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t status_handler(httpd_req_t *req);
static esp_err_t data_handler(httpd_req_t *req);
static esp_err_t data_json_handler(httpd_req_t *req);
static esp_err_t catch_all_handler(httpd_req_t *req);  // Catch-all for unknown URIs

// URI definitions
static const httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
};

static const httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
};

static const httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_handler,
    .user_ctx = NULL
};

static const httpd_uri_t data_json_uri = {
    .uri = "/data.json",
    .method = HTTP_GET,
    .handler = data_json_handler,
    .user_ctx = NULL
};

// Catch-all handler for unknown URIs (suppresses warnings from ESP-IDF components)
static const httpd_uri_t catch_all_uri = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = catch_all_handler,
    .user_ctx = NULL
};

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            connected_clients++;
            ESP_LOGI(TAG, "Client connected (AID=%d), total: %d", event->aid, connected_clients);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            if (connected_clients > 0) connected_clients--;
            ESP_LOGI(TAG, "Client disconnected (AID=%d), total: %d", event->aid, connected_clients);
        }
    }
}

// Initialize WiFi Access Point
static esp_err_t wifi_init_softap(void) {
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: SSID=%s, password=%s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    return ESP_OK;
}

// Start HTTP server
static esp_err_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_SERVER_PORT;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;
    config.stack_size = 8192;  // Larger stack for JSON generation
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &data_json_uri);
        // Register catch-all handler LAST (lowest priority) to suppress warnings
        httpd_register_uri_handler(server, &catch_all_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d (stack: %d)", config.server_port, config.stack_size);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

// HTTP Handlers
static esp_err_t root_handler(httpd_req_t *req) {
    const char *html = 
        "<!DOCTYPE html><html><head><title>GoldenForm</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:900px;margin:0 auto;padding:20px;background:#f5f5f5;}"
        "h1{color:#2563eb;}"
        ".card{background:white;padding:20px;border-radius:8px;margin:15px 0;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
        ".status-row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee;}"
        ".status-row:last-child{border-bottom:none;}"
        ".label{font-weight:bold;color:#555;}"
        ".value{font-family:monospace;color:#333;}"
        ".checksum{font-size:24px;color:#2563eb;font-weight:bold;font-family:monospace;}"
        "button{padding:12px 24px;font-size:16px;margin:5px;cursor:pointer;border:none;border-radius:5px;}"
        ".btn-primary{background:#2563eb;color:white;}"
        ".btn-secondary{background:#6b7280;color:white;}"
        ".verify{background:#dcfce7;padding:15px;border-radius:8px;margin-top:15px;}"
        ".verify h3{color:#166534;margin-top:0;}"
        "</style></head><body>"
        "<h1>GoldenForm Session Data</h1>"
        "<div class='card'>"
        "<h2>Session Status</h2>"
        "<div id='status'>Loading...</div>"
        "</div>"
        "<div class='card'>"
        "<h2>Actions</h2>"
        "<button class='btn-primary' onclick='fetchStatus()'>Refresh</button>"
        "<button class='btn-secondary' onclick='downloadData()'>Download Protobuf</button>"
        "<button class='btn-secondary' onclick='viewJson()'>View JSON</button>"
        "</div>"
        "<div class='card verify'>"
        "<h3>Data Verification</h3>"
        "<p>To verify data integrity, compare the <b>Checksum</b> shown above with the checksum printed in ESP32 serial monitor.</p>"
        "<p>If they match, all data was transferred correctly from SD card to this webpage.</p>"
        "</div>"
        "<script>"
        "function fetchStatus(){"
        "fetch('/status').then(r=>r.json()).then(d=>{"
        "document.getElementById('status').innerHTML="
        "\"<div class='status-row'><span class='label'>Syncing:</span><span class='value'>\"+(d.syncing?'YES - Ready':'NO')+'</span></div>'"
        "+\"<div class='status-row'><span class='label'>Samples:</span><span class='value'>\"+d.samples+'</span></div>'"
        "+\"<div class='status-row'><span class='label'>Size:</span><span class='value'>\"+(d.size_bytes/1024).toFixed(2)+' KB</span></div>'"
        "+\"<div class='status-row'><span class='label'>Checksum:</span><span class='checksum'>\"+d.checksum+'</span></div>';"
        "});}"
        "function downloadData(){"
        "if(!confirm('Download protobuf data file?')) return;"
        "const a=document.createElement('a');"
        "a.href='/data';"
        "a.download='session_data.pb';"
        "document.body.appendChild(a);"
        "a.click();"
        "document.body.removeChild(a);"
        "}"
        "function viewJson(){window.open('/data.json','_blank');}"
        "fetchStatus();setInterval(fetchStatus,2000);"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    char json_buf[512];
    size_t samples = 0;
    size_t size_bytes = 0;
    uint32_t checksum = 0;  // Simple checksum for verification
    
    // Get current streaming state
    if (is_syncing && stream_state.mutex && 
        xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Use total_samples_sent if transfer has started, otherwise use total_samples_available
        if (stream_state.total_samples_sent > 0) {
            samples = stream_state.total_samples_sent;
        } else if (stream_state.total_samples_available > 0) {
            samples = stream_state.total_samples_available;
        } else {
            // Estimate: each file typically contains ~1000-2000 samples
            // For accurate count, we'd need to scan files, but this is just for display
            samples = stream_state.file_count * 1500;  // Rough estimate
        }
        size_bytes = samples * sizeof(bno055_sample_t);
        
        // Simple checksum: file count + sample count (for basic verification)
        checksum = (uint32_t)(stream_state.file_count * 1000 + samples);
        
        xSemaphoreGive(stream_state.mutex);
    }
    
    // Match JavaScript field names: samples, size_bytes, checksum
    snprintf(json_buf, sizeof(json_buf),
        "{\"syncing\":%s,\"files\":%zu,\"samples\":%zu,\"size_bytes\":%zu,\"checksum\":%" PRIu32 ",\"transfer_complete\":%s,\"connected_clients\":%u}",
        is_syncing ? "true" : "false",
        stream_state.file_count,
        samples,
        size_bytes,
        checksum,
        (is_syncing && stream_state.transfer_complete) ? "true" : "false",
        connected_clients);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_buf, strlen(json_buf));
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    // Check if client is connected to GoldenForm WiFi
    if (!wifi_server_has_clients()) {
        ESP_LOGW(TAG, "Data request rejected: No client connected to GoldenForm WiFi");
        httpd_resp_send_custom_err(req, "503 Service Unavailable", "Not connected to GoldenForm WiFi. Please connect to SSID: GoldenForm");
        return ESP_FAIL;
    }
    
    if (!is_syncing || stream_state.file_count == 0) {
        ESP_LOGW(TAG, "Data request rejected: No data available");
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data available. Start sync first.");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Starting streaming transfer: %zu files, ~%zu samples available", 
             stream_state.file_count, stream_state.total_samples_available);
    
    httpd_resp_set_type(req, "application/x-protobuf");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=session_data.pb");
    
    // Lock streaming state
    if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire streaming mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // Reset transfer state for new request
    stream_state.current_file_idx = 0;
    stream_state.total_samples_sent = 0;
    stream_state.transfer_complete = false;
    
    // Open first file
    const char *mount_point = "/sdcard";
    char file_path[160];
    if (stream_state.file_count > 0 && stream_state.file_names[0]) {
        snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, stream_state.file_names[0]);
        stream_state.current_file = fopen(file_path, "rb");
        if (!stream_state.current_file) {
            ESP_LOGE(TAG, "Failed to open file: %s", file_path);
            xSemaphoreGive(stream_state.mutex);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Opened file: %s", stream_state.file_names[0]);
    } else {
        xSemaphoreGive(stream_state.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreGive(stream_state.mutex);
    
    // Stream data from SD card files in chunks
    while (true) {
        // Check if sync was stopped
        if (!is_syncing) {
            ESP_LOGW(TAG, "Sync interrupted during transfer");
            if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (stream_state.current_file) {
                    fclose(stream_state.current_file);
                    stream_state.current_file = NULL;
                }
                xSemaphoreGive(stream_state.mutex);
            }
            return ESP_FAIL;
        }
        
        // Lock to read chunk
        if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;  // Try again
        }
        
        // Read chunk from current file
        size_t samples_read = 0;
        esp_err_t read_err = ESP_OK;
        
        // Read up to STREAM_CHUNK_SAMPLES from current file
        // This loop handles protobuf batches that may be smaller or larger than our chunk buffer
        while (samples_read < STREAM_CHUNK_SAMPLES && stream_state.current_file) {
            size_t decoded = 0;
            size_t remaining_space = STREAM_CHUNK_SAMPLES - samples_read;
            
            // Only peek if we have limited space remaining (to avoid expensive peeks on every batch)
            // If we have plenty of space (>= MAX_BATCH_SAMPLES), just read directly
            if (remaining_space < MAX_BATCH_SAMPLES) {
                // Limited space - peek to check if batch will fit
                size_t batch_sample_count = 0;
                esp_err_t peek_err = protobuf_peek_batch_size(stream_state.current_file, &batch_sample_count);
                
                if (peek_err == ESP_ERR_NOT_FOUND) {
                    // EOF on current file - handle below
                    read_err = ESP_ERR_NOT_FOUND;
                    decoded = 0;
                } else if (peek_err == ESP_OK && batch_sample_count > remaining_space) {
                    // Batch won't fit in remaining space - break to send current chunk
                    // The batch will be read in the next iteration when we have full buffer space
                    ESP_LOGD(TAG, "Batch size %zu exceeds remaining space %zu - sending current chunk first", 
                             batch_sample_count, remaining_space);
                    break;
                } else {
                    // Batch will fit, or peek failed (try reading anyway)
                    // Read the batch (will handle errors in read_err)
                    read_err = protobuf_read_delimited(stream_state.current_file, 
                                                        stream_chunk_buffer + samples_read,
                                                        remaining_space,
                                                        &decoded);
                }
            } else {
                // Plenty of space - read directly without peeking (more efficient)
                read_err = protobuf_read_delimited(stream_state.current_file, 
                                                    stream_chunk_buffer + samples_read,
                                                    remaining_space,
                                                    &decoded);
            }
            
            if (read_err == ESP_ERR_NOT_FOUND) {
                // EOF on current file - move to next file
                fclose(stream_state.current_file);
                stream_state.current_file = NULL;
                stream_state.current_file_idx++;
                
                if (stream_state.current_file_idx >= stream_state.file_count) {
                    // All files processed
                    stream_state.transfer_complete = true;
                    xSemaphoreGive(stream_state.mutex);
                    break;
                }
                
                // Open next file
                snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, 
                        stream_state.file_names[stream_state.current_file_idx]);
                stream_state.current_file = fopen(file_path, "rb");
                if (!stream_state.current_file) {
                    ESP_LOGE(TAG, "Failed to open file: %s", file_path);
                    xSemaphoreGive(stream_state.mutex);
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "Opened next file: %s", stream_state.file_names[stream_state.current_file_idx]);
                continue;  // Read from new file
            }
            
            if (read_err == ESP_ERR_NO_MEM) {
                // Memory allocation failed for protobuf batch
                // protobuf_read_delimited already read the 4-byte length prefix but failed to allocate buffer
                // File pointer is now at start of protobuf data - we need to skip it
                ESP_LOGW(TAG, "Memory allocation failed for protobuf batch - skipping message to continue");
                
                // Skip the protobuf message to continue reading
                // Note: protobuf_read_delimited already consumed the 4-byte length prefix,
                // so we need to seek back 4 bytes, then use skip function
                long current_pos = ftell(stream_state.current_file);
                if (current_pos >= 4) {
                    // Seek back to re-read the length prefix
                    fseek(stream_state.current_file, -4, SEEK_CUR);
                    esp_err_t skip_err = protobuf_skip_delimited(stream_state.current_file);
                    if (skip_err == ESP_OK) {
                        ESP_LOGI(TAG, "Successfully skipped protobuf message after memory error");
                        continue;  // Try reading next message
                    } else {
                        ESP_LOGE(TAG, "Failed to skip protobuf message: %s", esp_err_to_name(skip_err));
                    }
                }
                
                // If skip failed, close file and move to next
                ESP_LOGE(TAG, "Cannot recover from memory allocation failure - moving to next file");
                fclose(stream_state.current_file);
                stream_state.current_file = NULL;
                stream_state.current_file_idx++;
                
                if (stream_state.current_file_idx >= stream_state.file_count) {
                    stream_state.transfer_complete = true;
                    xSemaphoreGive(stream_state.mutex);
                    break;
                }
                
                // Try next file
                snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, 
                        stream_state.file_names[stream_state.current_file_idx]);
                stream_state.current_file = fopen(file_path, "rb");
                if (!stream_state.current_file) {
                    ESP_LOGE(TAG, "Failed to open next file: %s", file_path);
                    xSemaphoreGive(stream_state.mutex);
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "Opened next file after memory error: %s", stream_state.file_names[stream_state.current_file_idx]);
                continue;
            }
            
            if (read_err != ESP_OK) {
                // Other errors (corrupted data, I/O error, etc.)
                ESP_LOGW(TAG, "Error reading protobuf batch: %s - attempting to continue", esp_err_to_name(read_err));
                // For corrupted data, we can't easily skip - log and try next file
                // Close current file and move to next
                fclose(stream_state.current_file);
                stream_state.current_file = NULL;
                stream_state.current_file_idx++;
                
                if (stream_state.current_file_idx >= stream_state.file_count) {
                    // No more files - transfer ends (with some data potentially lost)
                    ESP_LOGW(TAG, "Transfer ended due to read error - some data may be incomplete");
                    stream_state.transfer_complete = true;
                    xSemaphoreGive(stream_state.mutex);
                    break;
                }
                
                // Try next file
                snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, 
                        stream_state.file_names[stream_state.current_file_idx]);
                stream_state.current_file = fopen(file_path, "rb");
                if (!stream_state.current_file) {
                    ESP_LOGE(TAG, "Failed to open next file: %s", file_path);
                    xSemaphoreGive(stream_state.mutex);
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "Opened next file after error: %s", stream_state.file_names[stream_state.current_file_idx]);
                continue;  // Try reading from next file
            }
            
            // Successfully decoded samples
            if (decoded > 0) {
                samples_read += decoded;
                ESP_LOGD(TAG, "Read batch: %zu samples (total in chunk: %zu)", decoded, samples_read);
            } else {
                // No samples decoded but no error - should not happen, but break to avoid infinite loop
                ESP_LOGW(TAG, "protobuf_read_delimited returned OK but decoded 0 samples");
                break;
            }
        }
        
        xSemaphoreGive(stream_state.mutex);
        
        // If we have data, send it
        if (samples_read > 0) {
            esp_err_t send_err = httpd_resp_send_chunk(req, 
                (const char*)stream_chunk_buffer, 
                samples_read * sizeof(bno055_sample_t));
            
            if (send_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send chunk: %s", esp_err_to_name(send_err));
                return send_err;
            }
            
            // Update sent count
            if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                stream_state.total_samples_sent += samples_read;
                ESP_LOGI(TAG, "Sent chunk: %zu samples (total sent: %zu)", samples_read, stream_state.total_samples_sent);
                xSemaphoreGive(stream_state.mutex);
            }
            
            vTaskDelay(pdMS_TO_TICKS(1));  // Yield to prevent watchdog
        }
        
        // Check if transfer complete
        if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool complete = stream_state.transfer_complete;
            if (complete && stream_state.current_file) {
                fclose(stream_state.current_file);
                stream_state.current_file = NULL;
            }
            xSemaphoreGive(stream_state.mutex);
            
            if (complete) {
                break;  // All data sent
            }
        }
        
        // If no data read and not complete, check for errors
        if (samples_read == 0) {
            if (read_err == ESP_OK) {
                // No error but no data - wait a bit (shouldn't happen often)
                vTaskDelay(pdMS_TO_TICKS(10));
            } else if (read_err == ESP_ERR_NOT_FOUND) {
                // EOF handled above - this shouldn't reach here
                break;
            } else {
                // Error occurred - already handled above, but ensure we don't loop forever
                ESP_LOGW(TAG, "No data read due to error: %s", esp_err_to_name(read_err));
                vTaskDelay(pdMS_TO_TICKS(100));  // Wait before retry
            }
        }
    }
    
    // End chunked response
    httpd_resp_send_chunk(req, NULL, 0);
    
    if (xSemaphoreTake(stream_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "Data transfer complete: %zu samples sent", stream_state.total_samples_sent);
        stream_state.transfer_complete = true;
        xSemaphoreGive(stream_state.mutex);
    }
    
    return ESP_OK;
}

static esp_err_t data_json_handler(httpd_req_t *req) {
    // Check if client is connected to GoldenForm WiFi
    if (!wifi_server_has_clients()) {
        ESP_LOGW(TAG, "JSON data request rejected: No client connected to GoldenForm WiFi");
        httpd_resp_send_custom_err(req, "503 Service Unavailable", "Not connected to GoldenForm WiFi. Please connect to SSID: GoldenForm");
        return ESP_FAIL;
    }
    
    if (!is_syncing || stream_state.file_count == 0) {
        ESP_LOGW(TAG, "JSON data request rejected: No data available");
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data available");
        return ESP_FAIL;
    }
    
    // Stream full session as JSON (all files, all samples)
    size_t samples_sent = 0;

    httpd_resp_set_type(req, "application/json");

    // Send header (samples_in_response filled by client from data.length)
    char header[128];
    snprintf(header, sizeof(header), "{\"files\":%zu,\"data\":[", stream_state.file_count);
    httpd_resp_send_chunk(req, header, strlen(header));

    const char *mount_point = "/sdcard";
    char file_path[160];
    FILE *json_file = NULL;
    size_t file_idx = 0;

    while (file_idx < stream_state.file_count) {
        if (json_file == NULL) {
            snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, stream_state.file_names[file_idx]);
            json_file = fopen(file_path, "rb");
            if (!json_file) {
                ESP_LOGW(TAG, "Failed to open file for JSON: %s", file_path);
                file_idx++;
                continue;
            }
        }

        size_t decoded = 0;
        esp_err_t read_err = protobuf_read_delimited(json_file,
                                                      stream_chunk_buffer,
                                                      STREAM_CHUNK_SAMPLES,
                                                      &decoded);

        if (read_err == ESP_ERR_NOT_FOUND) {
            fclose(json_file);
            json_file = NULL;
            file_idx++;
            continue;
        }

        if (read_err != ESP_OK) {
            ESP_LOGW(TAG, "Error reading for JSON: %s", esp_err_to_name(read_err));
            if (json_file) {
                fclose(json_file);
                json_file = NULL;
            }
            break;
        }

        for (size_t i = 0; i < decoded; i++) {
            bno055_sample_t *s = &stream_chunk_buffer[i];
            char sample_buf[256];
            snprintf(sample_buf, sizeof(sample_buf),
                "%s{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                "\"lia_x\":%.3f,\"lia_y\":%.3f,\"lia_z\":%.3f,"
                "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
                "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f}",
                (samples_sent > 0) ? "," : "",
                (unsigned)s->t_ms, s->ax, s->ay, s->az,
                s->lia_x, s->lia_y, s->lia_z,
                s->gx, s->gy, s->gz,
                s->qw, s->qx, s->qy, s->qz);

            esp_err_t err = httpd_resp_send_chunk(req, sample_buf, strlen(sample_buf));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send JSON chunk: %s", esp_err_to_name(err));
                if (json_file) fclose(json_file);
                return err;
            }
            samples_sent++;
            if (samples_sent % 100 == 0) vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    if (json_file) {
        fclose(json_file);
    }

    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);

    ESP_LOGI(TAG, "JSON session sent: %zu samples (full session)", samples_sent);
    return ESP_OK;
}

// Catch-all handler for unknown URIs (suppresses warnings from ESP-IDF components)
static esp_err_t catch_all_handler(httpd_req_t *req) {
    // Silently ignore unknown URIs (like /auth/discovery from ESP-IDF components)
    // Only log at debug level to avoid spam
    ESP_LOGD(TAG, "Unknown URI requested: %s", req->uri);
    httpd_resp_send_404(req);
    return ESP_OK;
}

// Public API
esp_err_t wifi_server_init(void) {
    ESP_LOGI(TAG, "Initializing WiFi server...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize WiFi AP
    ret = wifi_init_softap();
    if (ret != ESP_OK) {
        current_state = WIFI_SERVER_STATE_ERROR;
        return ret;
    }

    // Start HTTP server
    ret = start_http_server();
    if (ret != ESP_OK) {
        current_state = WIFI_SERVER_STATE_ERROR;
        return ret;
    }

    current_state = WIFI_SERVER_STATE_RUNNING;
    ESP_LOGI(TAG, "WiFi server ready: http://192.168.4.1");
    return ESP_OK;
}

wifi_server_state_t wifi_server_get_state(void) {
    return current_state;
}

bool wifi_server_has_clients(void) {
    return connected_clients > 0;
}

esp_err_t wifi_server_start_sync(void) {
    if (is_syncing) {
        ESP_LOGW(TAG, "Already syncing");
        return ESP_OK;
    }
    
    // Check WiFi server state
    if (current_state != WIFI_SERVER_STATE_RUNNING) {
        ESP_LOGE(TAG, "WiFi server not running (state: %d)", current_state);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting data sync (streaming mode)...");
    
    // Initialize streaming state if needed
    esp_err_t ret = init_streaming_state();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Cleanup any previous state
    cleanup_streaming_state();
    
    // Give scheduler a chance to run
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Collect .pb file names from SD card (protobuf format)
    const char *mount_point = "/sdcard";
    DIR *dir = opendir(mount_point);
    if (!dir) {
        ESP_LOGE(TAG, "opendir(%s) failed", mount_point);
        return ESP_FAIL;
    }
    
    size_t names_cap = 16;
    stream_state.file_names = malloc(names_cap * sizeof(char *));
    if (!stream_state.file_names) {
        closedir(dir);
        return ESP_ERR_NO_MEM;
    }
    stream_state.file_count = 0;
    
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
#if defined(DT_REG)
        if (entry->d_type != DT_REG) continue;
#endif
        const char *n = entry->d_name;
        // Support both .pb (new) and .BIN/.bin (legacy) for backward compatibility
        bool is_protobuf = (strstr(n, ".pb") != NULL) || (strstr(n, ".PB") != NULL) ||
                          (strstr(n, ".BIN") != NULL) || (strstr(n, ".bin") != NULL);
        if (!is_protobuf) continue;
        
        size_t L = strnlen(n, 255);
        if (L == 0 || L >= 64) continue;
        
        if (stream_state.file_count == names_cap) {
            size_t new_cap = names_cap * 2;
            char **tmp = realloc(stream_state.file_names, new_cap * sizeof(char *));
            if (!tmp) break;
            stream_state.file_names = tmp;
            names_cap = new_cap;
        }
        
        stream_state.file_names[stream_state.file_count] = malloc(L + 1);
        if (!stream_state.file_names[stream_state.file_count]) break;
        memcpy(stream_state.file_names[stream_state.file_count], n, L);
        stream_state.file_names[stream_state.file_count][L] = '\0';
        stream_state.file_count++;
    }
    closedir(dir);
    
    if (stream_state.file_count == 0) {
        ESP_LOGW(TAG, "No .pb files found on SD card");
        ESP_LOGW(TAG, "Start a recording session first, then sync");
        cleanup_streaming_state();
        return ESP_ERR_NOT_FOUND;
    }
    
    // Sort files ascending by filename
    qsort(stream_state.file_names, stream_state.file_count, sizeof(char *), cmp_file_names);
    
    // Count actual samples by reading all files (accurate count, not estimate)
    // This ensures the status endpoint shows the correct count
    stream_state.total_samples_available = 0;
    char file_path[160];
    bno055_sample_t *count_buffer = malloc(256 * sizeof(bno055_sample_t));
    if (count_buffer) {
        for (size_t i = 0; i < stream_state.file_count; i++) {
            snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, stream_state.file_names[i]);
            FILE *f = fopen(file_path, "rb");
            if (f) {
                size_t file_samples = 0;
                while (true) {
                    size_t decoded = 0;
                    esp_err_t read_err = protobuf_read_delimited(f, count_buffer, 256, &decoded);
                    if (read_err == ESP_ERR_NOT_FOUND) {
                        break;  // EOF
                    }
                    if (read_err == ESP_OK && decoded > 0) {
                        file_samples += decoded;
                    } else {
                        break;  // Error or no more data
                    }
                }
                fclose(f);
                stream_state.total_samples_available += file_samples;
                ESP_LOGI(TAG, "File %s: %zu samples", stream_state.file_names[i], file_samples);
            }
        }
        free(count_buffer);
    } else {
        // Fallback to estimation if memory allocation fails
        ESP_LOGW(TAG, "Failed to allocate count buffer, using estimation");
        for (size_t i = 0; i < stream_state.file_count; i++) {
            snprintf(file_path, sizeof(file_path), "%s/%s", mount_point, stream_state.file_names[i]);
            FILE *f = fopen(file_path, "rb");
            if (f) {
                fseek(f, 0, SEEK_END);
                long file_size = ftell(f);
                fclose(f);
                if (file_size > 0) {
                    // Estimate: ~109 bytes per sample in protobuf format (more accurate)
                    stream_state.total_samples_available += (size_t)(file_size / 109);
                }
            }
        }
    }
    
    stream_state.current_file_idx = 0;
    stream_state.current_file = NULL;
    stream_state.total_samples_sent = 0;
    stream_state.transfer_complete = false;
    
    // Mark as syncing
    is_syncing = true;
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SYNC READY (Streaming Mode)");
    ESP_LOGI(TAG, "  Files: %zu", stream_state.file_count);
    ESP_LOGI(TAG, "  Mode: Stream from SD card (no RAM limit)");
    ESP_LOGI(TAG, "Connect to WiFi: GoldenForm (pw: goldenform123)");
    ESP_LOGI(TAG, "Open: http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    
    return ESP_OK;
}

esp_err_t wifi_server_stop_sync(void) {
    if (!is_syncing) {
        return ESP_OK;  // Already stopped
    }
    
    is_syncing = false;
    
    // Cleanup streaming state
    cleanup_streaming_state();
    
    // Log memory after cleanup
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Sync stopped - PSRAM: %zu bytes, Internal: %zu bytes", free_psram, free_internal);
    return ESP_OK;
}

bool wifi_server_is_syncing(void) {
    return is_syncing;
}

esp_err_t wifi_server_deinit(void) {
    wifi_server_stop_sync();
    
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    // Cleanup streaming resources
    cleanup_streaming_state();
    if (stream_chunk_buffer) {
        free(stream_chunk_buffer);
        stream_chunk_buffer = NULL;
    }
    if (stream_state.mutex) {
        vSemaphoreDelete(stream_state.mutex);
        stream_state.mutex = NULL;
    }
    
    current_state = WIFI_SERVER_STATE_IDLE;
    ESP_LOGI(TAG, "WiFi server stopped");
    return ESP_OK;
}
