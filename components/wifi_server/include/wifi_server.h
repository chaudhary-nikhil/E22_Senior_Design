#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFi Access Point Configuration
#define WIFI_AP_SSID              "GoldenForm"
#define WIFI_AP_PASSWORD          "goldenform123"
#define WIFI_AP_CHANNEL           1
#define WIFI_AP_MAX_CONNECTIONS   4

// HTTP Server Configuration
#define HTTP_SERVER_PORT          80

// WiFi server states
typedef enum {
    WIFI_SERVER_STATE_IDLE = 0,
    WIFI_SERVER_STATE_STARTING,
    WIFI_SERVER_STATE_RUNNING,
    WIFI_SERVER_STATE_ERROR
} wifi_server_state_t;

/**
 * @brief Initialize WiFi Access Point and HTTP server
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_init(void);

/**
 * @brief Get WiFi server state
 * @return wifi_server_state_t Current state
 */
wifi_server_state_t wifi_server_get_state(void);

/**
 * @brief Check if any client is connected to AP
 * @return true if at least one client connected
 */
bool wifi_server_has_clients(void);

/**
 * @brief Trigger data sync - reads from SD card and serves via HTTP
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_start_sync(void);

/**
 * @brief Stop data sync
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_stop_sync(void);

/**
 * @brief Check if currently syncing data
 * @return true if syncing
 */
bool wifi_server_is_syncing(void);

/**
 * @brief Check if data transfer has completed (all data sent to client)
 * @return true if transfer finished
 */
bool wifi_server_is_transfer_complete(void);

/**
 * @brief Deinitialize WiFi server
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_deinit(void);

#ifdef __cplusplus
}
#endif
