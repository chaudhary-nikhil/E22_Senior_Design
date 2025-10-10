#include "ble_service.h"
#include "sdkconfig.h"

#if CONFIG_FORMSYNC_ENABLE_BLE
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BLE_SERVICE";

// GATT interface and connection
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID      0xFF01
#define GATTS_DESCR_UUID     0x2902  // Client Characteristic Configuration
#define GATTS_NUM_HANDLE     8       // Increased for descriptor

static uint8_t adv_service_uuid128[16] = {
    0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint16_t gatts_if = 0;
static uint16_t conn_id = 0;
static uint16_t char_handle = 0;
static uint16_t descr_handle = 0;
static uint16_t service_handle = 0;
static bool is_connected = false;

// GAP event handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "📡 Advertising data set complete, starting advertising...");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "🎉 *** ADVERTISING STARTED SUCCESSFULLY ***");
                ESP_LOGI(TAG, "📱 Device 'FormSync' is now visible in Bluetooth!");
                ESP_LOGI(TAG, "🔍 Open Chrome and scan for 'FormSync'");
            } else {
                ESP_LOGE(TAG, "❌ Advertising start failed, status=%d", param->adv_start_cmpl.status);
            }
            break;
            
        default:
            break;
    }
}

// GATTS event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "📋 GATTS register, status %d, app_id %d", param->reg.status, param->reg.app_id);
            gatts_if = gatt_if;
            
            // Set device name
            esp_ble_gap_set_device_name("FormSync");
            ESP_LOGI(TAG, "✅ Device name set to 'FormSync'");
            
            // Configure advertising data
            esp_ble_gap_config_adv_data(&adv_data);
            ESP_LOGI(TAG, "✅ Advertising data configured");
            
            // Create service
            ESP_LOGI(TAG, "🔵 Creating GATT service...");
            esp_gatt_srvc_id_t service_id = {
                .is_primary = true,
                .id = {
                    .inst_id = 0,
                    .uuid = {
                        .len = ESP_UUID_LEN_16,
                        .uuid = {.uuid16 = GATTS_SERVICE_UUID}
                    }
                }
            };
            esp_ble_gatts_create_service(gatt_if, &service_id, GATTS_NUM_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "✅ Service created, status %d, service_handle %d", 
                     param->create.status, param->create.service_handle);
            
            service_handle = param->create.service_handle;
            
            // Start service
            ESP_LOGI(TAG, "🔵 Starting GATT service...");
            esp_ble_gatts_start_service(service_handle);
            ESP_LOGI(TAG, "✅ Service started");
            
            // Add characteristic with notification support
            ESP_LOGI(TAG, "🔵 Adding IMU characteristic with notification support...");
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_CHAR_UUID}
            };
            
            esp_attr_value_t attr_val = {
                .attr_max_len = 28,
                .attr_len = 0,
                .attr_value = NULL
            };
            
            esp_attr_control_t control = {
                .auto_rsp = ESP_GATT_AUTO_RSP,
            };
            
            esp_ble_gatts_add_char(service_handle,
                                   &char_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   &attr_val, &control);
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "✅ Characteristic added, status %d, attr_handle %d", 
                     param->add_char.status, param->add_char.attr_handle);
            char_handle = param->add_char.attr_handle;
            
            // Add CCCD (Client Characteristic Configuration Descriptor) for notifications
            ESP_LOGI(TAG, "🔵 Adding CCCD descriptor for notifications...");
            esp_bt_uuid_t descr_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = GATTS_DESCR_UUID}
            };
            
            uint8_t cccd_value[2] = {0x00, 0x00};
            esp_attr_value_t descr_val = {
                .attr_max_len = 2,
                .attr_len = 2,
                .attr_value = cccd_value
            };
            
            esp_attr_control_t descr_control = {
                .auto_rsp = ESP_GATT_AUTO_RSP,
            };
            
            esp_ble_gatts_add_char_descr(service_handle, &descr_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                         &descr_val, &descr_control);
            break;
            
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(TAG, "✅ CCCD descriptor added, status %d, attr_handle %d",
                     param->add_char_descr.status, param->add_char_descr.attr_handle);
            descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "🎉 GATT service setup complete!");
            ESP_LOGI(TAG, "📡 Ready to accept BLE connections");
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "🎉 *** CLIENT CONNECTED ***");
            ESP_LOGI(TAG, "📱 Connection ID: %d", param->connect.conn_id);
            ESP_LOGI(TAG, "📱 Remote address: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(param->connect.remote_bda));
            conn_id = param->connect.conn_id;
            gatts_if = gatt_if;
            is_connected = true;
            
            // Update connection parameters for better performance
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x10;  // 20ms
            conn_params.max_int = 0x20;  // 40ms
            conn_params.latency = 0;
            conn_params.timeout = 400;   // 4s
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGW(TAG, "⚠️  Client disconnected, restarting advertising");
            is_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        default:
            break;
    }
}

// Initialize BLE
esp_err_t ble_init_if_enabled(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "🔵 CONFIG_FORMSYNC_ENABLE_BLE is ENABLED");
    ESP_LOGI(TAG, "🔵 Starting Bluetooth controller initialization...");
    
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_LOGI(TAG, "🔵 Calling esp_bt_controller_init...");
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "❌ Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Bluetooth controller initialized");
    
    // Enable BT controller in BLE mode
    ESP_LOGI(TAG, "🔵 Enabling BT controller in BLE mode...");
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "❌ Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ BT controller enabled");
    
    // Initialize Bluedroid
    ESP_LOGI(TAG, "🔵 Initializing Bluedroid stack...");
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "❌ Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Bluedroid initialized");
    
    ESP_LOGI(TAG, "🔵 Enabling Bluedroid...");
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "❌ Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ Bluedroid enabled");
    
    // Register callbacks
    ESP_LOGI(TAG, "🔵 Registering GATTS callback...");
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "❌ GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ GATTS callback registered");
    
    ESP_LOGI(TAG, "🔵 Registering GAP callback...");
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "❌ GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ GAP callback registered");
    
    // Register application
    ESP_LOGI(TAG, "🔵 Registering GATT application...");
    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "❌ GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✅ GATT application registered");
    
    // Set MTU
    ESP_LOGI(TAG, "🔵 Setting MTU to 500...");
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret) {
        ESP_LOGW(TAG, "⚠️  Set local MTU failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "✅ MTU set");
    }
    
    ESP_LOGI(TAG, "🎉 BLE service initialized successfully!");
    ESP_LOGI(TAG, "📡 Device will advertise as 'FormSync'");
    return ESP_OK;
}

// Send IMU data via BLE
esp_err_t ble_send_imu_data(uint32_t t_ms, float ax, float ay, float az, float gx, float gy, float gz) {
    if (!is_connected || char_handle == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Pack data into buffer (28 bytes: 1 uint32 + 6 floats)
    uint8_t data[28];
    memcpy(&data[0], &t_ms, 4);
    memcpy(&data[4], &ax, 4);
    memcpy(&data[8], &ay, 4);
    memcpy(&data[12], &az, 4);
    memcpy(&data[16], &gx, 4);
    memcpy(&data[20], &gy, 4);
    memcpy(&data[24], &gz, 4);
    
    // Send notification
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, char_handle,
                                                 sizeof(data), data, false);
    if (ret != ESP_OK) {
        static int err_count = 0;
        if (err_count++ % 100 == 0) {
            ESP_LOGW(TAG, "Failed to send BLE notification: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    return ESP_OK;
}

// Start BLE advertising
void ble_advertise(void) {
    esp_ble_gap_start_advertising(&adv_params);
}

#else
// BLE disabled - stub implementation
esp_err_t ble_init_if_enabled(void) {
    return ESP_OK;
}

esp_err_t ble_send_imu_data(uint32_t t_ms, float ax, float ay, float az, float gx, float gy, float gz) {
    (void)t_ms; (void)ax; (void)ay; (void)az; (void)gx; (void)gy; (void)gz;
    return ESP_OK;
}

void ble_advertise(void) {
    // No-op
}
#endif
