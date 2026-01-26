#include "bus_i2c.h"
#include "esp_log.h"
#include "soc/i2c_reg.h"
#include "hal/i2c_ll.h"
#include "esp_clk_tree.h"

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
    // BNO055 datasheet: clock stretching can be up to 500μs during fusion calculations
    // We set the maximum hardware timeout to ensure reliable operation
    if (err == ESP_OK) {
        // Set timeout to maximum hardware value
        // ESP32-S3: I2C_LL_MAX_TIMEOUT = 0x1F (31)
        // Timeout formula: timeout_cycles = 2^(timeout_reg + 1) APB clock cycles
        // At 80MHz APB: 2^32 / 80MHz = ~53.7ms (plenty of margin)
        esp_err_t timeout_err = i2c_set_timeout(port, I2C_LL_MAX_TIMEOUT);
        if (timeout_err != ESP_OK) {
            ESP_LOGW(TAG_I2C, "Failed to set I2C timeout to max value, using default");
        } else {
            // Read back and verify the timeout setting
            int timeout_val = 0;
            esp_err_t read_err = i2c_get_timeout(port, &timeout_val);
            if (read_err == ESP_OK) {
                // Calculate actual timeout in microseconds
                // For ESP32-S3: timeout = 2^(timeout_reg+1) / APB_CLK_FREQ
                uint32_t apb_freq = 0;
                esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &apb_freq);
                if (apb_freq > 0) {
                    // timeout_cycles = 2^(timeout_val + 1)
                    uint64_t timeout_cycles = 1ULL << (timeout_val + 1);
                    uint32_t timeout_us = (uint32_t)((timeout_cycles * 1000000ULL) / apb_freq);
                    ESP_LOGI(TAG_I2C, "I2C timeout configured:");
                    ESP_LOGI(TAG_I2C, "  Register value: %d (max: %d)", timeout_val, I2C_LL_MAX_TIMEOUT);
                    ESP_LOGI(TAG_I2C, "  APB clock: %lu Hz", apb_freq);
                    ESP_LOGI(TAG_I2C, "  Calculated timeout: %lu μs (~%lu ms)", timeout_us, timeout_us / 1000);
                    ESP_LOGI(TAG_I2C, "  BNO055 requirement: ~500 μs (clock stretching)");
                } else {
                    ESP_LOGI(TAG_I2C, "I2C timeout set to register value %d", timeout_val);
                }
            } else {
                ESP_LOGI(TAG_I2C, "I2C timeout set to max value (%d)", I2C_LL_MAX_TIMEOUT);
            }
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
