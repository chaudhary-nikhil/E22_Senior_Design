#pragma once
#include "esp_err.h"
#ifdef __cplusplus
extern "C" {
#endif
esp_err_t storage_init_if_enabled(void); // no-op unless FORMSYNC_ENABLE_STORAGE=y
#ifdef __cplusplus
}
#endif
