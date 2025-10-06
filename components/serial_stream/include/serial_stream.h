#ifndef SERIAL_STREAM_H
#define SERIAL_STREAM_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
esp_err_t serial_stream_init(void);
esp_err_t serial_stream_send_data(const char* data);

#ifdef __cplusplus
}
#endif

#endif // SERIAL_STREAM_H
