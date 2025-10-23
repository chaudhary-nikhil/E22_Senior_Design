#include "wifi_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cJSON.h>

static const char *TAG = "WIFI_SERVER";

// Global state variables
static wifi_server_state_t current_state = WIFI_SERVER_STATE_IDLE;
static session_state_t session_state = SESSION_STATE_IDLE;
static httpd_handle_t server = NULL;
static wifi_status_callback_t status_callback = NULL;
static session_status_callback_t session_callback = NULL;

// Session data management
static uint8_t *session_buffer = NULL;
static uint32_t session_buffer_size = 0;
static uint32_t session_data_count = 0;
static SemaphoreHandle_t session_mutex = NULL;

// Protobuf-like data structure for efficient storage
typedef struct {
    uint32_t timestamp_ms;
    float ax, ay, az;
    float gx, gy, gz;
    float qw, qx, qy, qz;
    uint8_t cal_status; // Packed calibration values
    uint8_t reserved[4]; // Padding
} __attribute__((packed)) imu_data_t;

#define IMU_DATA_SIZE sizeof(imu_data_t)

// HTTP request handlers
static esp_err_t root_handler(httpd_req_t *req);
static esp_err_t start_logging_handler(httpd_req_t *req);
static esp_err_t stop_logging_handler(httpd_req_t *req);
static esp_err_t session_status_handler(httpd_req_t *req);
static esp_err_t session_data_handler(httpd_req_t *req);
static esp_err_t create_visualization_handler(httpd_req_t *req);
static esp_err_t sessions_list_handler(httpd_req_t *req);

// HTTP URI handlers
static const httpd_uri_t root_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t start_logging_uri = {
    .uri       = "/start_logging",
    .method    = HTTP_POST,
    .handler   = start_logging_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t stop_logging_uri = {
    .uri       = "/stop_logging",
    .method    = HTTP_POST,
    .handler   = stop_logging_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t session_status_uri = {
    .uri       = "/session_status",
    .method    = HTTP_GET,
    .handler   = session_status_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t session_data_uri = {
    .uri       = "/data/*",
    .method    = HTTP_GET,
    .handler   = session_data_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t create_visualization_uri = {
    .uri       = "/create_visualization",
    .method    = HTTP_POST,
    .handler   = create_visualization_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t sessions_list_uri = {
    .uri       = "/sessions",
    .method    = HTTP_GET,
    .handler   = sessions_list_handler,
    .user_ctx  = NULL
};

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected, AID=%d", event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station disconnected, AID=%d", event->aid);
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

    ESP_LOGI(TAG, "WiFi AP initialized. SSID:%s password:%s",
             WIFI_AP_SSID, WIFI_AP_PASSWORD);

    return ESP_OK;
}

// Start HTTP server
static esp_err_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_SERVER_PORT;
    config.max_open_sockets = 7;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &start_logging_uri);
        httpd_register_uri_handler(server, &stop_logging_uri);
        httpd_register_uri_handler(server, &session_status_uri);
        httpd_register_uri_handler(server, &session_data_uri);
        httpd_register_uri_handler(server, &create_visualization_uri);
        httpd_register_uri_handler(server, &sessions_list_uri);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

// HTTP handler implementations
static esp_err_t root_handler(httpd_req_t *req) {
    // Simple API status response instead of HTML interface
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "service", "GoldenForm WiFi Session Logger");
    cJSON_AddStringToObject(json, "status", "running");
    cJSON_AddStringToObject(json, "version", "1.0");
    cJSON_AddStringToObject(json, "endpoints", "/start_logging, /stop_logging, /session_status, /create_visualization");
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t start_logging_handler(httpd_req_t *req) {
    esp_err_t ret = wifi_server_start_logging();
    
    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddStringToObject(json, "status", "logging_started");
        cJSON_AddNumberToObject(json, "data_points", session_data_count);
    } else {
        cJSON_AddStringToObject(json, "status", "error");
        cJSON_AddStringToObject(json, "error", "Failed to start logging");
    }
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t stop_logging_handler(httpd_req_t *req) {
    esp_err_t ret = wifi_server_stop_logging();
    
    cJSON *json = cJSON_CreateObject();
    if (ret == ESP_OK) {
        cJSON_AddStringToObject(json, "status", "logging_stopped");
        cJSON_AddNumberToObject(json, "data_points", session_data_count);
        cJSON_AddNumberToObject(json, "buffer_size", session_buffer_size);
    } else {
        cJSON_AddStringToObject(json, "status", "error");
        cJSON_AddStringToObject(json, "error", "Failed to stop logging");
    }
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t session_status_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "logging_active", (session_state == SESSION_STATE_LOGGING));
    cJSON_AddNumberToObject(json, "data_points", session_data_count);
    cJSON_AddNumberToObject(json, "buffer_size", session_buffer_size);
    cJSON_AddStringToObject(json, "session_state", 
                           (session_state == SESSION_STATE_LOGGING) ? "logging" : 
                           (session_state == SESSION_STATE_STOPPED) ? "stopped" : "idle");
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t session_data_handler(httpd_req_t *req) {
    // For now, return the raw protobuf data
    if (session_buffer && session_buffer_size > 0) {
        httpd_resp_set_type(req, "application/octet-stream");
        httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=session_data.bin");
        httpd_resp_send(req, (char*)session_buffer, session_buffer_size);
    } else {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No session data available");
    }
    return ESP_OK;
}

static esp_err_t create_visualization_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    
    if (session_state == SESSION_STATE_STOPPED && session_buffer && session_data_count > 0) {
        cJSON_AddStringToObject(json, "status", "visualization_started");
        cJSON_AddNumberToObject(json, "data_points", session_data_count);
        cJSON_AddNumberToObject(json, "buffer_size", session_buffer_size);
        cJSON_AddStringToObject(json, "message", "Data ready for transmission to laptop");
        
        ESP_LOGI(TAG, "Visualization requested: %d data points, %d bytes", session_data_count, session_buffer_size);
    } else {
        cJSON_AddStringToObject(json, "status", "error");
        cJSON_AddStringToObject(json, "error", "No session data available for visualization");
    }
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

static esp_err_t sessions_list_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateArray();
    cJSON *session = cJSON_CreateObject();
    cJSON_AddStringToObject(session, "filename", "current_session.bin");
    cJSON_AddNumberToObject(session, "data_points", session_data_count);
    cJSON_AddNumberToObject(session, "size_bytes", session_buffer_size);
    cJSON_AddItemToArray(json, session);
    
    char *json_string = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(json);
    return ESP_OK;
}

// Public API implementations
esp_err_t wifi_server_init(wifi_status_callback_t status_cb, session_status_callback_t session_cb) {
    ESP_LOGI(TAG, "Initializing WiFi server");
    
    status_callback = status_cb;
    session_callback = session_cb;
    
    // Initialize NVS
    ESP_LOGI(TAG, "Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "NVS initialized successfully");

    // Initialize TCP/IP adapter
    ESP_LOGI(TAG, "Initializing TCP/IP...");
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCP/IP initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Event loop creation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "TCP/IP initialized successfully");

    // Initialize WiFi Access Point
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    ret = wifi_init_softap();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi Access Point initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "WiFi Access Point initialized successfully");

    // Start HTTP server
    ESP_LOGI(TAG, "Starting HTTP server...");
    ret = start_http_server();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "HTTP server started successfully");

    // Initialize session mutex
    ESP_LOGI(TAG, "Creating session mutex...");
    session_mutex = xSemaphoreCreateMutex();
    if (session_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create session mutex");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Session mutex created successfully");

    current_state = WIFI_SERVER_STATE_RUNNING;
    if (status_callback) {
        status_callback(current_state);
    }

    ESP_LOGI(TAG, "✅ WiFi server initialized successfully!");
    ESP_LOGI(TAG, "🌐 Connect to WiFi: %s (password: %s)", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ESP_LOGI(TAG, "🌐 Open browser to: http://192.168.4.1");

    return ESP_OK;
}

esp_err_t wifi_server_start_logging(void) {
    if (session_state == SESSION_STATE_LOGGING) {
        ESP_LOGW(TAG, "Already logging");
        return ESP_OK;
    }

    if (xSemaphoreTake(session_mutex, portMAX_DELAY) == pdTRUE) {
        // Allocate session buffer
        if (session_buffer) {
            free(session_buffer);
        }
        
        session_buffer_size = PROTOBUF_MAX_SESSION_SIZE;
        session_buffer = malloc(session_buffer_size);
        if (!session_buffer) {
            ESP_LOGE(TAG, "Failed to allocate session buffer");
            xSemaphoreGive(session_mutex);
            return ESP_FAIL;
        }
        
        session_data_count = 0;
        session_state = SESSION_STATE_LOGGING;
        
        xSemaphoreGive(session_mutex);
        
        ESP_LOGI(TAG, "Started logging session");
        
        if (session_callback) {
            session_callback(session_state);
        }
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t wifi_server_stop_logging(void) {
    if (session_state != SESSION_STATE_LOGGING) {
        ESP_LOGW(TAG, "Not currently logging");
        return ESP_OK;
    }

    if (xSemaphoreTake(session_mutex, portMAX_DELAY) == pdTRUE) {
        session_state = SESSION_STATE_STOPPED;
        xSemaphoreGive(session_mutex);
        
        ESP_LOGI(TAG, "Stopped logging session. Data points: %d, Buffer size: %d bytes", 
                 session_data_count, session_buffer_size);
        
        if (session_callback) {
            session_callback(session_state);
        }
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t wifi_server_add_data_point(uint32_t timestamp_ms, 
                                    float ax, float ay, float az,
                                    float gx, float gy, float gz,
                                    float qw, float qx, float qy, float qz,
                                    uint8_t sys_cal, uint8_t gyro_cal, uint8_t accel_cal, uint8_t mag_cal) {
    if (session_state != SESSION_STATE_LOGGING || !session_buffer) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(session_mutex, portMAX_DELAY) == pdTRUE) {
        // Check if we have space for another data point
        uint32_t required_size = (session_data_count + 1) * IMU_DATA_SIZE;
        if (required_size > session_buffer_size) {
            ESP_LOGW(TAG, "Session buffer full, cannot add more data");
            xSemaphoreGive(session_mutex);
            return ESP_ERR_NO_MEM;
        }

        // Pack calibration status
        uint8_t cal_status = (sys_cal & 0x3) | ((gyro_cal & 0x3) << 2) | 
                           ((accel_cal & 0x3) << 4) | ((mag_cal & 0x3) << 6);

        // Create data point
        imu_data_t data_point = {
            .timestamp_ms = timestamp_ms,
            .ax = ax, .ay = ay, .az = az,
            .gx = gx, .gy = gy, .gz = gz,
            .qw = qw, .qx = qx, .qy = qy, .qz = qz,
            .cal_status = cal_status,
            .reserved = {0, 0, 0, 0}
        };

        // Store in buffer
        memcpy(session_buffer + (session_data_count * IMU_DATA_SIZE), 
               &data_point, IMU_DATA_SIZE);
        
        session_data_count++;
        
        xSemaphoreGive(session_mutex);
        
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

session_state_t wifi_server_get_session_state(void) {
    return session_state;
}

uint32_t wifi_server_get_session_count(void) {
    return session_data_count;
}

wifi_server_state_t wifi_server_get_state(void) {
    return current_state;
}

uint8_t wifi_server_get_connected_count(void) {
    // This would require additional implementation to track connected stations
    return 0; // Placeholder
}

esp_err_t wifi_server_deinit(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    
    if (session_buffer) {
        free(session_buffer);
        session_buffer = NULL;
    }
    
    if (session_mutex) {
        vSemaphoreDelete(session_mutex);
        session_mutex = NULL;
    }
    
    esp_wifi_stop();
    esp_wifi_deinit();
    
    current_state = WIFI_SERVER_STATE_IDLE;
    session_state = SESSION_STATE_IDLE;
    
    ESP_LOGI(TAG, "WiFi server deinitialized");
    return ESP_OK;
}
