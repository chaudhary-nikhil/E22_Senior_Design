#include "storage.h"
#include "sdkconfig.h"
esp_err_t storage_init_if_enabled(void) {
#if CONFIG_FORMSYNC_ENABLE_STORAGE
    // TODO: mount SD/MMC or SPIFFS
    return ESP_OK;
#else
    return ESP_OK;
#endif
}
