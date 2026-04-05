#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFi Access Point Configuration
#define WIFI_AP_SSID_BASE         "GoldenForm"
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
 * @brief Prepare WiFi subsystem (NVS, TCP/IP, event loop) without starting the AP.
 *        Call once at boot.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_init(void);

/**
 * @brief Start the WiFi AP and HTTP server (makes the SSID visible).
 *        Call when entering sync mode.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_start_ap(void);

/**
 * @brief SSID string used for the AP (after wifi_server_start_ap), including device suffix.
 */
const char *wifi_server_get_ap_ssid(void);

/**
 * @brief Stop the WiFi AP and HTTP server (SSID disappears).
 *        Call when leaving sync mode.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_stop_ap(void);

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
 * @brief Check if WiFi AP is currently active (SSID visible)
 * @return true if AP is running
 */
bool wifi_server_is_ap_active(void);

/**
 * @brief Protobuf `device_role` field for logged samples: 0 = wrist_right, 4 = wrist_left.
 *        Value is stored in NVS (user-settable via POST /api/user_config); Kconfig is only the factory default.
 */
uint32_t goldenform_device_role_pb(void);

/**
 * @brief Set wrist role from dashboard strings "wrist_left" or "wrist_right"; persists to NVS.
 */
esp_err_t goldenform_set_device_role_str(const char *role);

/**
 * @brief Called when POST /api/registration_done completes successfully (after HTTP response is sent).
 *        Firmware uses this to clear registration linger state, stop the status LED blink, and shut down the AP.
 */
void wifi_server_set_registration_done_callback(void (*cb)(void));

/**
 * @brief Deinitialize WiFi server
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_server_deinit(void);

#ifdef __cplusplus
}
#endif
