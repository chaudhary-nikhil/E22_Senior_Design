#pragma once
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t ble_init_if_enabled(void); // no-op unless FORMSYNC_ENABLE_BLE=y
#ifdef __cplusplus
}
#endif
