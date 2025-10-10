#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize BLE service (no-op unless FORMSYNC_ENABLE_BLE=y)
esp_err_t ble_init_if_enabled(void);

// Send IMU data via BLE notification
esp_err_t ble_send_imu_data(uint32_t t_ms, float ax, float ay, float az, float gx, float gy, float gz);

// Start BLE advertising
void ble_advertise(void);

#ifdef __cplusplus
}
#endif
