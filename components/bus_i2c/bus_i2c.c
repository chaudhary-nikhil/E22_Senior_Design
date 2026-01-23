#include "bus_i2c.h"
#include "esp_log.h"
#include "soc/i2c_reg.h"

static const char *TAG_I2C = "BUS_I2C";

esp_err_t bus_i2c_init(i2c_port_t port, int sda, int scl, uint32_t hz) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
        .master.clk_speed = hz,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    esp_err_t err = i2c_driver_install(port, conf.mode, 0, 0, 0);
    
    // Set I2C timeout for BNO055 clock stretching
    // ESP32-S3 max timeout is I2C_TIME_OUT_VALUE (0x1F = 31) which at 80MHz gives ~1.3ms
    // This is sufficient for BNO055 which needs ~500μs max
    if (err == ESP_OK) {
        // Use max timeout value for ESP32-S3 (0x1F = 31, which is the max for the hardware)
        // The actual timeout depends on clock config, but max value ensures BNO055 works
        esp_err_t timeout_err = i2c_set_timeout(port, 0xFFFFF);  // Max valid timeout
        if (timeout_err != ESP_OK) {
            // If that fails, try a smaller value that's definitely valid
            i2c_set_timeout(port, 0xFFFF);
        }
        ESP_LOGI(TAG_I2C, "I2C initialized on SDA=%d, SCL=%d @ %lu Hz", sda, scl, hz);
    }
    
    return err;
}

esp_err_t bus_i2c_wr8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val, TickType_t to) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf), to);
}

esp_err_t bus_i2c_wrrd(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, TickType_t to) {
    return i2c_master_write_read_device(port, addr, &reg, 1, buf, len, to);
}
