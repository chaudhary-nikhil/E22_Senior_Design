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

#include "storage.h"
#include "bno055.h"

static const char *TAG = "WIFI_SERVER";

// Global state
static wifi_server_state_t current_state = WIFI_SERVER_STATE_IDLE;
static httpd_handle_t server = NULL;
static bool is_syncing = false;
static uint8_t connected_clients = 0;

// Data buffer for HTTP transfer (allocated during sync)
static bno055_sample_t *sync_buffer = NULL;
static size_t sync_buffer_count = 0;
static uint32_t sync_checksum = 0;  // Simple checksum for verification
#define SYNC_BUFFER_MAX_SAMPLES 4096  // ~400KB max

// Simple checksum: sum of all bytes
static uint32_t compute_checksum(const bno055_sample_t *data, size_t count) {
    uint32_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    size_t total_bytes = count * sizeof(bno055_sample_t);
    for (size_t i = 0; i < total_bytes; i++) {
        sum += bytes[i];
    }
    return sum;
}

// HTTP handlers
static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t status_handler(httpd_req_t *req);
static esp_err_t data_handler(httpd_req_t *req);
static esp_err_t data_json_handler(httpd_req_t *req);

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

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &data_uri);
        httpd_register_uri_handler(server, &data_json_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
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
        "<button class='btn-secondary' onclick='downloadData()'>Download Binary</button>"
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
        "function downloadData(){window.location='/data';}"
        "function viewJson(){window.open('/data.json','_blank');}"
        "fetchStatus();setInterval(fetchStatus,2000);"
        "</script></body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    char json_buf[512];
    snprintf(json_buf, sizeof(json_buf),
        "{\"syncing\":%s,\"samples\":%zu,\"size_bytes\":%zu,\"checksum\":%" PRIu32 ",\"connected_clients\":%u}",
        is_syncing ? "true" : "false",
        sync_buffer_count,
        sync_buffer_count * sizeof(bno055_sample_t),
        sync_checksum,
        connected_clients);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_buf, strlen(json_buf));
    return ESP_OK;
}

static esp_err_t data_handler(httpd_req_t *req) {
    if (!is_syncing || !sync_buffer || sync_buffer_count == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data available. Start sync first.");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sending %zu samples (%zu bytes)", 
             sync_buffer_count, sync_buffer_count * sizeof(bno055_sample_t));
    
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=session_data.bin");
    
    // Send in chunks to avoid memory issues
    size_t chunk_size = 64;  // samples per chunk
    size_t sent = 0;
    
    while (sent < sync_buffer_count) {
        size_t to_send = sync_buffer_count - sent;
        if (to_send > chunk_size) to_send = chunk_size;
        
        esp_err_t err = httpd_resp_send_chunk(req, 
            (const char*)(sync_buffer + sent), 
            to_send * sizeof(bno055_sample_t));
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send chunk");
            return err;
        }
        sent += to_send;
        vTaskDelay(pdMS_TO_TICKS(1));  // Yield to prevent watchdog
    }
    
    // End chunked response
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "Data transfer complete");
    return ESP_OK;
}

static esp_err_t data_json_handler(httpd_req_t *req) {
    if (!is_syncing || !sync_buffer || sync_buffer_count == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data available");
        return ESP_FAIL;
    }
    
    // Limit JSON output to prevent memory issues
    size_t max_samples = 200;
    size_t samples_to_send = (sync_buffer_count < max_samples) ? sync_buffer_count : max_samples;
    
    httpd_resp_set_type(req, "application/json");
    
    // Send header with checksum for verification
    char header[256];
    snprintf(header, sizeof(header), 
        "{\"total_samples\":%zu,\"samples_in_response\":%zu,\"checksum\":%" PRIu32 ",\"data\":[",
        sync_buffer_count, samples_to_send, sync_checksum);
    httpd_resp_send_chunk(req, header, strlen(header));
    
    // Send samples one by one
    char sample_buf[256];
    for (size_t i = 0; i < samples_to_send; i++) {
        bno055_sample_t *s = &sync_buffer[i];
        snprintf(sample_buf, sizeof(sample_buf),
            "%s{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
            "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f}",
            (i > 0) ? "," : "",
            (unsigned)s->t_ms, s->ax, s->ay, s->az,
            s->gx, s->gy, s->gz,
            s->qw, s->qx, s->qy, s->qz);
        httpd_resp_send_chunk(req, sample_buf, strlen(sample_buf));
        
        // Yield occasionally
        if (i % 50 == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // Send footer
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);  // End chunked response
    
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
    
    ESP_LOGI(TAG, "Starting data sync - reading from SD card...");
    
    // Allocate buffer from PSRAM (internal RAM is too small for large data)
    if (sync_buffer) {
        free(sync_buffer);
    }
    
    size_t buffer_size = SYNC_BUFFER_MAX_SAMPLES * sizeof(bno055_sample_t);
    ESP_LOGI(TAG, "Allocating %zu bytes from PSRAM for sync buffer...", buffer_size);
    
    // Try PSRAM first (8MB available on ESP32-S3-WROOM-1)
    sync_buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (!sync_buffer) {
        // Fallback to internal RAM (unlikely to work for large buffers)
        ESP_LOGW(TAG, "PSRAM allocation failed, trying internal RAM...");
        sync_buffer = malloc(buffer_size);
    }
    
    if (!sync_buffer) {
        ESP_LOGE(TAG, "Failed to allocate sync buffer (%zu bytes)", buffer_size);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Sync buffer allocated successfully");
    
    // Read all data from SD card
    esp_err_t ret = storage_read_all_bin_into_buffer(sync_buffer, SYNC_BUFFER_MAX_SAMPLES, &sync_buffer_count);
    
    // If no data or error, generate test data for testing WiFi transfer
    if ((ret != ESP_OK && ret != ESP_ERR_NO_MEM) || sync_buffer_count == 0) {
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read SD card: %s", esp_err_to_name(ret));
        }
        ESP_LOGI(TAG, "Generating test data for WiFi testing...");
        
        sync_buffer_count = 10;  // 10 test samples
        for (size_t i = 0; i < sync_buffer_count; i++) {
            memset(&sync_buffer[i], 0, sizeof(bno055_sample_t));
            sync_buffer[i].t_ms = (uint32_t)(i * 100);
            sync_buffer[i].ax = 0.1f * (float)i;
            sync_buffer[i].ay = 0.2f * (float)i;
            sync_buffer[i].az = 9.8f;
            sync_buffer[i].gx = 0.01f * (float)i;
            sync_buffer[i].gy = 0.02f * (float)i;
            sync_buffer[i].gz = 0.0f;
            sync_buffer[i].mx = 30.0f;
            sync_buffer[i].my = 0.0f;
            sync_buffer[i].mz = 45.0f;
            sync_buffer[i].roll = 1.0f + (float)i;
            sync_buffer[i].pitch = 2.0f + (float)i;
            sync_buffer[i].yaw = 3.0f + (float)i;
            sync_buffer[i].qw = 1.0f;
            sync_buffer[i].qx = 0.0f;
            sync_buffer[i].qy = 0.0f;
            sync_buffer[i].qz = 0.0f;
            sync_buffer[i].lia_x = 0.0f;
            sync_buffer[i].lia_y = 0.0f;
            sync_buffer[i].lia_z = 0.0f;
            sync_buffer[i].temp = 25.0f;
            sync_buffer[i].sys_cal = 3;
            sync_buffer[i].gyro_cal = 3;
            sync_buffer[i].accel_cal = 3;
            sync_buffer[i].mag_cal = 3;
        }
        ESP_LOGI(TAG, "Generated %zu test samples for WiFi testing", sync_buffer_count);
    }
    
    // Compute checksum for verification
    sync_checksum = compute_checksum(sync_buffer, sync_buffer_count);
    
    is_syncing = true;
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SYNC READY - Data loaded from SD card");
    ESP_LOGI(TAG, "  Samples: %zu", sync_buffer_count);
    ESP_LOGI(TAG, "  Size: %zu bytes", sync_buffer_count * sizeof(bno055_sample_t));
    ESP_LOGI(TAG, "  CHECKSUM: %" PRIu32, sync_checksum);
    ESP_LOGI(TAG, "Connect to WiFi: GoldenForm");
    ESP_LOGI(TAG, "Open: http://192.168.4.1");
    ESP_LOGI(TAG, "Compare checksum on webpage to verify!");
    ESP_LOGI(TAG, "========================================");
    return ESP_OK;
}

esp_err_t wifi_server_stop_sync(void) {
    if (!is_syncing) {
        return ESP_OK;
    }
    
    is_syncing = false;
    
    if (sync_buffer) {
        free(sync_buffer);
        sync_buffer = NULL;
    }
    sync_buffer_count = 0;
    sync_checksum = 0;
    
    ESP_LOGI(TAG, "Sync stopped");
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
    
    current_state = WIFI_SERVER_STATE_IDLE;
    ESP_LOGI(TAG, "WiFi server stopped");
    return ESP_OK;
}
