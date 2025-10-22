#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ble_service.h"
#include "sdkconfig.h"

// Global variables for session data transmission
static bool is_transmitting_session = false;
static float transmission_progress = 0.0f;
static size_t total_session_size = 0;
static size_t transmitted_session_size = 0;

// Bluetooth configuration
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // 7.5ms
    .max_interval = 0x0010, // 20ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,    // 20ms
    .adv_int_max = 0x40,    // 40ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT service configuration
#define GATTS_TAG "GATTS_GOLDEN_FORM"
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

// Service and characteristic UUIDs
static uint16_t golden_form_service_id = 0;
static uint16_t golden_form_char_handle = 0;

// Connection state
static ble_stream_state_t current_state = BLE_STREAM_STATE_IDLE;
static uint8_t connected_clients = 0;
static uint16_t conn_id = 0;

// Callback function
static ble_connection_callback_t connection_callback = NULL;

// Data queue for sending
static QueueHandle_t data_queue = NULL;
#define DATA_QUEUE_SIZE 10
#define MAX_JSON_LENGTH 512

typedef struct {
    char data[MAX_JSON_LENGTH];
    uint16_t length;
} queue_data_t;

// GATT profile structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

// Forward declarations
static void ble_data_sender_task(void *pvParameters);
static esp_err_t send_json_packet(const char* json_data, uint16_t length);

/**
 * @brief GATT profile event handler
 */
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:{
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GOLDEN_FORM_SERVICE_UUID;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name("GoldenForm");
        if (set_dev_name_ret != ESP_OK) {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    }
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0x01;
        rsp.attr_value.value[1] = 0x02;
        rsp.attr_value.value[2] = 0x03;
        rsp.attr_value.value[3] = 0x04;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle %d, value len %d, value :", param->write.handle, param->write.len);
        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
        if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.is_prep == false) {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001) {
                ESP_LOGI(GATTS_TAG, "notify enable");
                uint8_t notify_data[15];
                for (int i = 0; i < sizeof(notify_data); ++i) {
                    notify_data[i] = i % 0xff;
                }
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                            sizeof(notify_data), notify_data, false);
            } else if (descr_value == 0x0000) {
                ESP_LOGI(GATTS_TAG, "notify disable");
            } else {
                ESP_LOGE(GATTS_TAG, "unknown descr value");
            }
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        break;
    }
    case ESP_GATTS_MTU_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    }
    case ESP_GATTS_CONF_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    }
    case ESP_GATTS_START_EVT: {
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->start.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GOLDEN_FORM_CHAR_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                                        NULL, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_GATT_OK){
            ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x",i,prf_char[i]);
            }
        }
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    }
    case ESP_GATTS_DELETE_EVT: {
        ESP_LOGI(GATTS_TAG, "DELETE_EVT");
        break;
    }
    case ESP_GATTS_STOP_EVT: {
        ESP_LOGI(GATTS_TAG, "SERVICE_STOP_EVT");
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        conn_id = param->connect.conn_id;
        connected_clients = 1;
        current_state = BLE_STREAM_STATE_CONNECTED;
        if (connection_callback) {
            connection_callback(current_state);
        }
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        connected_clients = 0;
        current_state = BLE_STREAM_STATE_ADVERTISING;
        if (connection_callback) {
            connection_callback(current_state);
        }
        break;
    }
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

/**
 * @brief GAP event handler
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started successfully");
            current_state = BLE_STREAM_STATE_ADVERTISING;
            if (connection_callback) {
                connection_callback(current_state);
            }
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BLE_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

/**
 * @brief Task to send queued data via Bluetooth
 */
static void ble_data_sender_task(void *pvParameters) {
    queue_data_t queue_item;
    
    ESP_LOGI(TAG, "Bluetooth data sender task started");
    
    while (1) {
        if (xQueueReceive(data_queue, &queue_item, portMAX_DELAY) == pdTRUE) {
            if (current_state == BLE_STREAM_STATE_CONNECTED) {
                send_json_packet(queue_item.data, queue_item.length);
            } else {
                ESP_LOGW(TAG, "Not connected, dropping data packet");
            }
        }
    }
}

/**
 * @brief Send JSON data packet via BLE notifications
 */
static esp_err_t send_json_packet(const char* json_data, uint16_t length) {
    if (current_state != BLE_STREAM_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Split large JSON into multiple packets if needed
    uint16_t offset = 0;
    while (offset < length) {
        uint16_t packet_size = (length - offset > BLE_MAX_DATA_SIZE) ? BLE_MAX_DATA_SIZE : (length - offset);
        
        esp_err_t ret = esp_ble_gatts_send_indicate(
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
            conn_id,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle,
            packet_size,
            (uint8_t*)(json_data + offset),
            false
        );
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send BLE packet: %s", esp_err_to_name(ret));
            return ret;
        }
        
        offset += packet_size;
        
        // Minimal delay between packets for faster transmission
        if (offset < length) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    
    return ESP_OK;
}

esp_err_t ble_stream_init(const char* device_name, ble_connection_callback_t callback) {
    ESP_LOGI(TAG, "Initializing Bluetooth stream");
    
    // Store callback
    connection_callback = callback;
    
    // Initialize Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_profile_a_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register GATT application
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create data queue
    data_queue = xQueueCreate(DATA_QUEUE_SIZE, sizeof(queue_data_t));
    if (data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create data queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create data sender task
    BaseType_t task_ret = xTaskCreate(ble_data_sender_task, "ble_data_sender", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create data sender task");
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Bluetooth stream initialized successfully");
    return ESP_OK;
}

esp_err_t ble_stream_send_data(const char* data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    queue_data_t queue_item;
    uint16_t length = strlen(data);
    
    if (length >= MAX_JSON_LENGTH) {
        ESP_LOGE(TAG, "Data too long: %d bytes (max %d)", length, MAX_JSON_LENGTH - 1);
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(queue_item.data, data, MAX_JSON_LENGTH - 1);
    queue_item.data[MAX_JSON_LENGTH - 1] = '\0';
    queue_item.length = length;
    
    BaseType_t ret = xQueueSend(data_queue, &queue_item, pdMS_TO_TICKS(100));
    if (ret != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue data (queue full)");
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

ble_stream_state_t ble_stream_get_state(void) {
    return current_state;
}

uint8_t ble_stream_get_connected_count(void) {
    return connected_clients;
}

esp_err_t ble_stream_deinit(void) {
    ESP_LOGI(TAG, "Deinitializing Bluetooth stream");
    
    // Stop advertising
    esp_ble_gap_stop_advertising();
    
    // Disable Bluedroid
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    
    // Disable Bluetooth controller
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    // Clean up queue
    if (data_queue) {
        vQueueDelete(data_queue);
        data_queue = NULL;
    }
    
    current_state = BLE_STREAM_STATE_IDLE;
    connected_clients = 0;
    
    ESP_LOGI(TAG, "Bluetooth stream deinitialized");
    return ESP_OK;
}

esp_err_t ble_init_if_enabled(void) {
#if CONFIG_GOLDENFORM_ENABLE_BLE
    // Initialize Bluetooth stream with default settings
    return ble_stream_init("GoldenForm", NULL);
#else
    return ESP_OK;
#endif
}

/**
 * @brief Send large session data via Bluetooth in optimized batches
 */
esp_err_t ble_stream_send_session_data(const char* session_data, size_t data_size) {
    if (session_data == NULL || data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (current_state != BLE_STREAM_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Cannot send session data - not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (is_transmitting_session) {
        ESP_LOGW(TAG, "Already transmitting session data");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (data_size > MAX_SESSION_DATA_SIZE) {
        ESP_LOGE(TAG, "Session data too large: %zu bytes (max %d)", data_size, MAX_SESSION_DATA_SIZE);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting session data transmission: %zu bytes", data_size);
    
    // Initialize transmission tracking
    is_transmitting_session = true;
    total_session_size = data_size;
    transmitted_session_size = 0;
    transmission_progress = 0.0f;
    
    // Send data in batches
    size_t offset = 0;
    uint32_t batch_count = 0;
    
    while (offset < data_size) {
        size_t batch_size = (data_size - offset > BLE_BATCH_SIZE) ? BLE_BATCH_SIZE : (data_size - offset);
        
        ESP_LOGD(TAG, "Sending batch %lu: offset=%zu, size=%zu", batch_count, offset, batch_size);
        
        // Send this batch
        esp_err_t ret = send_json_packet(session_data + offset, batch_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send batch %lu: %s", batch_count, esp_err_to_name(ret));
            is_transmitting_session = false;
            return ret;
        }
        
        // Update progress
        offset += batch_size;
        transmitted_session_size = offset;
        transmission_progress = (float)transmitted_session_size / (float)total_session_size * 100.0f;
        
        ESP_LOGI(TAG, "Session transmission progress: %.1f%% (%zu/%zu bytes)", 
                 transmission_progress, transmitted_session_size, total_session_size);
        
        batch_count++;
        
        // Minimal delay between batches for faster transmission
        if (offset < data_size) {
            vTaskDelay(pdMS_TO_TICKS(BLE_BATCH_DELAY_MS));
        }
    }
    
    // Transmission complete
    is_transmitting_session = false;
    transmission_progress = 100.0f;
    
    ESP_LOGI(TAG, "Session data transmission complete: %zu bytes in %lu batches", 
             data_size, batch_count);
    
    return ESP_OK;
}

/**
 * @brief Send session data from file path
 */
esp_err_t ble_stream_send_session_file(const char* file_path) {
    if (file_path == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (current_state != BLE_STREAM_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Cannot send session file - not connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Reading session file: %s", file_path);
    
    // Open file
    FILE* file = fopen(file_path, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open session file: %s", file_path);
        return ESP_ERR_NOT_FOUND;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (file_size <= 0 || file_size > MAX_SESSION_DATA_SIZE) {
        ESP_LOGE(TAG, "Invalid file size: %ld bytes", file_size);
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Allocate buffer for file content
    char* file_content = malloc(file_size + 1);
    if (file_content == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for file content");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    
    // Read file content
    size_t bytes_read = fread(file_content, 1, file_size, file);
    fclose(file);
    
    if (bytes_read != file_size) {
        ESP_LOGE(TAG, "Failed to read complete file: %zu/%ld bytes", bytes_read, file_size);
        free(file_content);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    file_content[file_size] = '\0'; // Null terminate
    
    // Send the session data
    esp_err_t ret = ble_stream_send_session_data(file_content, file_size);
    
    // Clean up
    free(file_content);
    
    return ret;
}

/**
 * @brief Get transmission progress for large data
 */
float ble_stream_get_transmission_progress(void) {
    return transmission_progress;
}

/**
 * @brief Check if currently transmitting large data
 */
bool ble_stream_is_transmitting(void) {
    return is_transmitting_session;
}
