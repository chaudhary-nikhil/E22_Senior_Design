#pragma once
#include "esp_err.h"
#include "ble_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct ble_gatt_profile ble_gatt_profile_t;
typedef struct ble_data_queue ble_data_queue_t;

/**
 * @brief Initialize GATT server and services
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_gatt_init(void);

/**
 * @brief Initialize data transmission queue
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_data_queue_init(void);

/**
 * @brief Send data packet via BLE notifications
 * 
 * @param json_data JSON string to send
 * @param length Length of JSON string
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ble_send_packet(const char* json_data, uint16_t length);

/**
 * @brief GATT profile event handler
 * 
 * @param event GATT event type
 * @param gatts_if GATT interface
 * @param param Event parameters
 */
void ble_gatt_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/**
 * @brief GAP event handler
 * 
 * @param event GAP event type
 * @param param Event parameters
 */
void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/**
 * @brief Data sender task
 * 
 * @param pvParameters Task parameters
 */
void ble_data_sender_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
