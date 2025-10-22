#pragma once
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// Bluetooth service UUIDs for GoldenForm tracker
#define GOLDEN_FORM_SERVICE_UUID        0x180F  // Battery Service (reusing for our data)
#define GOLDEN_FORM_CHAR_UUID           0x2A19  // Battery Level Characteristic (reusing for our data)

// Maximum data packet size for BLE (20 bytes for notifications)
#define BLE_MAX_PACKET_SIZE           20
#define BLE_MAX_DATA_SIZE             (BLE_MAX_PACKET_SIZE - 3)  // Account for GATT overhead
#define MAX_JSON_LENGTH               512   // Maximum single JSON data length
#define MAX_SESSION_DATA_SIZE         50000 // Maximum session data size (50KB)
#define BLE_BATCH_SIZE                2000  // Increased batch size for faster transmission
#define BLE_BATCH_DELAY_MS            10    // Reduced delay for faster transmission

// Bluetooth stream states
typedef enum {
    BLE_STREAM_STATE_IDLE = 0,
    BLE_STREAM_STATE_ADVERTISING,
    BLE_STREAM_STATE_CONNECTED,
    BLE_STREAM_STATE_DISCONNECTED
} ble_stream_state_t;

// Callback function type for connection events
typedef void (*ble_connection_callback_t)(ble_stream_state_t state);

/**
 * @brief Initialize Bluetooth stream with Bluedroid stack
 * 
 * @param device_name Name to advertise (max 20 characters)
 * @param connection_callback Optional callback for connection state changes
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_stream_init(const char* device_name, ble_connection_callback_t connection_callback);

/**
 * @brief Send JSON data via Bluetooth
 * 
 * This function will automatically split large JSON strings into multiple BLE packets
 * if needed. Each packet is sent as a notification to connected clients.
 * 
 * @param data JSON string to send (null-terminated)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t ble_stream_send_data(const char* data);

/**
 * @brief Send large session data via Bluetooth in batches
 * 
 * This function is optimized for sending large JSON session files (5-10 minutes of data).
 * It automatically splits the data into manageable batches and sends them with appropriate
 * delays to avoid overwhelming the receiver.
 * 
 * @param session_data Large JSON string containing complete session data
 * @param data_size Size of the session data in bytes
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if not connected
 */
esp_err_t ble_stream_send_session_data(const char* session_data, size_t data_size);

/**
 * @brief Send session data from file path
 * 
 * Reads session data from a file and sends it via Bluetooth in batches.
 * This is the preferred method for sending stored session data.
 * 
 * @param file_path Path to the session JSON file
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_stream_send_session_file(const char* file_path);

/**
 * @brief Get transmission progress for large data
 * 
 * @return float Progress percentage (0.0 to 100.0)
 */
float ble_stream_get_transmission_progress(void);

/**
 * @brief Check if currently transmitting large data
 * 
 * @return bool true if transmitting, false otherwise
 */
bool ble_stream_is_transmitting(void);

/**
 * @brief Get current Bluetooth connection state
 * 
 * @return ble_stream_state_t Current state
 */
ble_stream_state_t ble_stream_get_state(void);

/**
 * @brief Get number of connected clients
 * 
 * @return uint8_t Number of connected clients (0 or 1 for BLE)
 */
uint8_t ble_stream_get_connected_count(void);

/**
 * @brief Deinitialize Bluetooth stream
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_stream_deinit(void);

esp_err_t ble_init_if_enabled(void); // no-op unless GOLDENFORM_ENABLE_BLE=y

#ifdef __cplusplus
}
#endif
