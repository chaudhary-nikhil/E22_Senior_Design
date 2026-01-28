#include "bus_i2c.h"
#include "esp_log.h"

static const char *TAG_I2C = "BUS_I2C";

esp_err_t bus_i2c_init(i2c_port_t port, int sda, int scl, uint32_t hz) {
    // Validate GPIO numbers (ESP32-S3 supports GPIO 0-48, but some are input-only)
    // Common valid I2C pins on ESP32-S3: 1-21, 26-48 (avoid strapping pins: 0, 3, 45, 46)
    if (sda < 1 || sda > 48 || scl < 1 || scl > 48) {
        ESP_LOGE(TAG_I2C, "Invalid GPIO numbers: SDA=%d, SCL=%d (must be 1-48)", sda, scl);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG_I2C, "Initializing I2C port %d: SDA=GPIO%d, SCL=GPIO%d, speed=%lu Hz", 
             port, sda, scl, (unsigned long)hz);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = hz,
    };
    
    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "i2c_param_config failed: %s (SDA=%d, SCL=%d)", esp_err_to_name(ret), sda, scl);
        return ret;
    }
    esp_err_t err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set I2C timeout for BNO055 clock stretching (critical fix)
    // Based on GitHub discussion: BNO055 can stretch clock up to 500μs
    err = i2c_set_timeout(port, 500000); // Timeout for BNO055 clock stretching: 500,000 APB cycles (~500μs at 80MHz APB clock)
    if (err != ESP_OK) {
        ESP_LOGW(TAG_I2C, "i2c_set_timeout failed: %s (continuing anyway)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG_I2C, "I2C initialized with 500μs timeout for BNO055 compatibility");
    }
    
    ESP_LOGI(TAG_I2C, "I2C port %d initialized successfully", port);
    return ESP_OK;
}

esp_err_t bus_i2c_wr8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val, TickType_t to) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf), to);
}

esp_err_t bus_i2c_wrrd(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, TickType_t to) {
    return i2c_master_write_read_device(port, addr, &reg, 1, buf, len, to);
}
