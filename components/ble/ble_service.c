#include "ble_service.h"
#include "sdkconfig.h"
esp_err_t ble_init_if_enabled(void) {
#if CONFIG_FORMSYNC_ENABLE_BLE
    // TODO: Initialize NimBLE and GATT services
    // For the template, return OK without doing anything.
    return ESP_OK;
#else
    return ESP_OK;
#endif
}
