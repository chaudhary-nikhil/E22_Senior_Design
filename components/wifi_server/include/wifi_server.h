#pragma once
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi Access Point Configuration
#define WIFI_AP_SSID              "GoldenForm"
#define WIFI_AP_PASSWORD          "goldenform123"
#define WIFI_AP_CHANNEL           1
#define WIFI_AP_MAX_CONNECTIONS   4
#define WIFI_AP_IP                "192.168.4.1"
#define WIFI_AP_GATEWAY           "192.168.4.1"
#define WIFI_AP_NETMASK           "255.255.255.0"

// HTTP Server Configuration
#define HTTP_SERVER_PORT          80
#define MAX_HTTP_RECV_BUFFER      2048
#define MAX_HTTP_OUTPUT_BUFFER    4096

// Protobuf Configuration
#define PROTOBUF_MAX_SESSION_SIZE 100000  // 100KB max session size (about 2000 data points)
#define PROTOBUF_BATCH_SIZE       1024     // 1KB batches for transmission

// WiFi server states
typedef enum {
    WIFI_SERVER_STATE_IDLE = 0,
    WIFI_SERVER_STATE_STARTING,
    WIFI_SERVER_STATE_RUNNING,
    WIFI_SERVER_STATE_ERROR
} wifi_server_state_t;

// Session logging states
typedef enum {
    SESSION_STATE_IDLE = 0,
    SESSION_STATE_LOGGING,
    SESSION_STATE_STOPPED
} session_state_t;

// Callback function types
typedef void (*wifi_status_callback_t)(wifi_server_state_t state);
typedef void (*session_status_callback_t)(session_state_t state);

/**
 * @brief Initialize WiFi Access Point and HTTP server
 * 
 * @param status_callback Optional callback for WiFi status changes
 * @param session_callback Optional callback for session state changes
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_init(wifi_status_callback_t status_callback, session_status_callback_t session_callback);

/**
 * @brief Start logging session (stores data in protobuf buffers)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_start_logging(void);

/**
 * @brief Stop logging session and prepare data for transmission
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_stop_logging(void);

/**
 * @brief Add IMU data point to current session buffer
 * 
 * @param timestamp_ms Timestamp in milliseconds
 * @param ax, ay, az Accelerometer data
 * @param gx, gy, gz Gyroscope data
 * @param qw, qx, qy, qz Quaternion data
 * @param sys_cal, gyro_cal, accel_cal, mag_cal Calibration values
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_add_data_point(uint32_t timestamp_ms, 
                                    float ax, float ay, float az,
                                    float gx, float gy, float gz,
                                    float qw, float qx, float qy, float qz,
                                    uint8_t sys_cal, uint8_t gyro_cal, uint8_t accel_cal, uint8_t mag_cal);

/**
 * @brief Get current session status
 * 
 * @return session_state_t Current session state
 */
session_state_t wifi_server_get_session_state(void);

/**
 * @brief Get number of data points in current session
 * 
 * @return uint32_t Number of data points
 */
uint32_t wifi_server_get_session_count(void);

/**
 * @brief Get WiFi server state
 * 
 * @return wifi_server_state_t Current WiFi server state
 */
wifi_server_state_t wifi_server_get_state(void);

/**
 * @brief Get number of connected clients
 * 
 * @return uint8_t Number of connected clients
 */
uint8_t wifi_server_get_connected_count(void);

/**
 * @brief Deinitialize WiFi server
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_deinit(void);

#ifdef __cplusplus
}
#endif
