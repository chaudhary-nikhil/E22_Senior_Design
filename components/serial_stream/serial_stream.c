#include <stdio.h>
#include <string.h>
#include "esp_log.h"

#include "serial_stream.h"

static const char *TAG = "SERIAL_STREAM";

// Initialize serial streaming (using default printf output)
esp_err_t serial_stream_init(void)
{
    ESP_LOGI(TAG, "Serial stream initialized (using printf)");
    return ESP_OK;
}

// Send JSON data via serial (using printf)
esp_err_t serial_stream_send_data(const char* data)
{
    // Send data with newline for easy parsing
    printf("%s\n", data);
    fflush(stdout);  // Ensure data is sent immediately
    
    ESP_LOGI(TAG, "Sent: %s", data);
    return ESP_OK;
}
