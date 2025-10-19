#include "bno055.h"
#include "bus_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BNO055";

// Helper function to read 8-bit register
static esp_err_t bno055_read8(int port, uint8_t addr, uint8_t reg, uint8_t *data) {
    return bus_i2c_wrrd(port, addr, reg, data, 1, pdMS_TO_TICKS(1000)); // Increased timeout for BNO055
}

// Helper function to write 8-bit register
static esp_err_t bno055_write8(int port, uint8_t addr, uint8_t reg, uint8_t data) {
    return bus_i2c_wr8(port, addr, reg, data, pdMS_TO_TICKS(1000)); // Increased timeout for BNO055
}

// Helper function to read 16-bit register (little endian)
static esp_err_t bno055_read16(int port, uint8_t addr, uint8_t reg, int16_t *data) {
    uint8_t buffer[2];
    esp_err_t err = bus_i2c_wrrd(port, addr, reg, buffer, 2, pdMS_TO_TICKS(1000)); // Increased timeout for BNO055
    if (err == ESP_OK) {
        *data = (int16_t)((buffer[1] << 8) | buffer[0]);
    }
    return err;
}

esp_err_t bno055_init(int port, uint8_t addr) {
    ESP_LOGI(TAG, "Initializing BNO055 at address 0x%02X", addr);
    
    // Wait for BNO055 to boot up (critical for proper initialization)
    ESP_LOGI(TAG, "Waiting for BNO055 to boot up...");
    vTaskDelay(pdMS_TO_TICKS(650)); // BNO055 needs ~650ms to boot
    
    // Try both I2C addresses if the first one fails
    uint8_t addresses[] = {addr, (addr == BNO055_ADDR_A) ? BNO055_ADDR_B : BNO055_ADDR_A};
    uint8_t working_addr = 0;
    uint8_t chip_id;
    esp_err_t err = ESP_FAIL;
    
    for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
        uint8_t test_addr = addresses[addr_idx];
        ESP_LOGI(TAG, "Trying BNO055 at address 0x%02X", test_addr);
        
        int retry_count = 0;
        const int max_retries = 20; // Increased retries
        
        do {
            err = bno055_read8(port, test_addr, BNO055_CHIP_ID_ADDR, &chip_id);
            if (err == ESP_OK && chip_id == BNO055_ID) {
                working_addr = test_addr;
                ESP_LOGI(TAG, "BNO055 found at address 0x%02X (after %d retries)", test_addr, retry_count);
                break;
            }
            
            retry_count++;
            if (retry_count < max_retries) {
                ESP_LOGW(TAG, "BNO055 not ready at 0x%02X, retry %d/%d (waiting 200ms)", test_addr, retry_count, max_retries);
                vTaskDelay(pdMS_TO_TICKS(200)); // Increased delay between retries
            }
        } while (retry_count < max_retries);
        
        if (err == ESP_OK && chip_id == BNO055_ID) {
            break;
        }
    }
    
    if (err != ESP_OK || chip_id != BNO055_ID) {
        ESP_LOGE(TAG, "Failed to find BNO055 at both addresses 0x28 and 0x29");
        ESP_LOGE(TAG, "Check wiring: SDA=21, SCL=22, VCC=3.3V, GND=GND, pull-ups=4.7kΩ");
        ESP_LOGE(TAG, "Check power supply and I2C connections");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Update the address to the working one
    addr = working_addr;
    
    // Reset the device
    err = bno055_reset(port, addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset BNO055: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for reset to complete (BNO055 needs time after reset)
    ESP_LOGI(TAG, "Waiting for BNO055 reset to complete...");
    vTaskDelay(pdMS_TO_TICKS(650));
    
    // Set to config mode
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set power mode to normal
    err = bno055_write8(port, addr, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power mode: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set page to 0
    err = bno055_write8(port, addr, BNO055_PAGE_ID_ADDR, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set page: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set units (m/s², rad/s, μT, degrees)
    err = bno055_write8(port, addr, BNO055_UNIT_SEL_ADDR, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set units: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set to NDOF mode (fusion mode using all sensors)
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change (BNO055 needs time to switch modes)
    ESP_LOGI(TAG, "Waiting for NDOF mode to activate...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "BNO055 initialized successfully in NDOF mode");
    return ESP_OK;
}

esp_err_t bno055_set_operation_mode(int port, uint8_t addr, bno055_opmode_t mode) {
    return bno055_write8(port, addr, BNO055_OPR_MODE_ADDR, (uint8_t)mode);
}

esp_err_t bno055_reset(int port, uint8_t addr) {
    // Write reset command to SYS_TRIGGER register
    return bno055_write8(port, addr, BNO055_SYS_TRIGGER_ADDR, 0x20);
}

esp_err_t bno055_start_calibration(int port, uint8_t addr) {
    ESP_LOGI(TAG, "Starting BNO055 calibration process...");
    
    // Set to config mode for calibration
    esp_err_t err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode for calibration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change
    vTaskDelay(pdMS_TO_TICKS(25));
    
    // Set to NDOF mode to start calibration
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode for calibration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change
    vTaskDelay(pdMS_TO_TICKS(25));
    
    ESP_LOGI(TAG, "BNO055 calibration started - perform calibration movements");
    return ESP_OK;
}

esp_err_t bno055_save_calibration_data(int port, uint8_t addr, uint8_t *cal_data) {
    ESP_LOGI(TAG, "Saving BNO055 calibration data...");
    
    // Set to config mode to access calibration registers
    esp_err_t err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode for saving calibration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change
    vTaskDelay(pdMS_TO_TICKS(25));
    
    // Read calibration data (22 bytes total)
    // Accelerometer offsets (6 bytes): 0x55-0x5A
    // Magnetometer offsets (6 bytes): 0x5B-0x60  
    // Gyroscope offsets (6 bytes): 0x61-0x66
    // Accelerometer radius (2 bytes): 0x67-0x68
    // Magnetometer radius (2 bytes): 0x69-0x6A
    
    uint8_t *offset = cal_data;
    
    // Read accelerometer offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_read8(port, addr, 0x55 + i, &offset[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accel offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Read magnetometer offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_read8(port, addr, 0x5B + i, &offset[6 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read mag offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Read gyroscope offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_read8(port, addr, 0x61 + i, &offset[12 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Read accelerometer radius
    for (int i = 0; i < 2; i++) {
        err = bno055_read8(port, addr, 0x67 + i, &offset[18 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accel radius %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Read magnetometer radius
    for (int i = 0; i < 2; i++) {
        err = bno055_read8(port, addr, 0x69 + i, &offset[20 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read mag radius %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    ESP_LOGI(TAG, "Calibration data saved successfully");
    return ESP_OK;
}

esp_err_t bno055_load_calibration_data(int port, uint8_t addr, const uint8_t *cal_data) {
    ESP_LOGI(TAG, "Loading BNO055 calibration data...");
    
    // Set to config mode to write calibration registers
    esp_err_t err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode for loading calibration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change
    vTaskDelay(pdMS_TO_TICKS(25));
    
    const uint8_t *offset = cal_data;
    
    // Write accelerometer offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_write8(port, addr, 0x55 + i, offset[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write accel offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Write magnetometer offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_write8(port, addr, 0x5B + i, offset[6 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write mag offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Write gyroscope offsets
    for (int i = 0; i < 6; i++) {
        err = bno055_write8(port, addr, 0x61 + i, offset[12 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write gyro offset %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Write accelerometer radius
    for (int i = 0; i < 2; i++) {
        err = bno055_write8(port, addr, 0x67 + i, offset[18 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write accel radius %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Write magnetometer radius
    for (int i = 0; i < 2; i++) {
        err = bno055_write8(port, addr, 0x69 + i, offset[20 + i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write mag radius %d: %s", i, esp_err_to_name(err));
            return err;
        }
    }
    
    // Set back to NDOF mode
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode after loading calibration: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for mode change
    vTaskDelay(pdMS_TO_TICKS(25));
    
    ESP_LOGI(TAG, "Calibration data loaded successfully");
    return ESP_OK;
}

esp_err_t bno055_get_calibration_status(int port, uint8_t addr, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t cal_status;
    esp_err_t err = bno055_read8(port, addr, BNO055_CALIB_STAT_ADDR, &cal_status);
    if (err != ESP_OK) {
        return err;
    }
    
    *sys = (cal_status >> 6) & 0x03;
    *gyro = (cal_status >> 4) & 0x03;
    *accel = (cal_status >> 2) & 0x03;
    *mag = cal_status & 0x03;
    
    return ESP_OK;
}

esp_err_t bno055_read_sample(int port, uint8_t addr, bno055_sample_t *out) {
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use relative timestamp starting from 0 for this session
    static uint32_t session_start_time = 0;
    if (session_start_time == 0) {
        session_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    out->t_ms = (xTaskGetTickCount() * portTICK_PERIOD_MS) - session_start_time;
    
    // Read accelerometer data
    int16_t ax_raw, ay_raw, az_raw;
    if (bno055_read16(port, addr, BNO055_ACCEL_DATA_X_LSB_ADDR, &ax_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_ACCEL_DATA_Y_LSB_ADDR, &ay_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_ACCEL_DATA_Z_LSB_ADDR, &az_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert to m/s² (LSB = 1 m/s²)
    out->ax = (float)ax_raw;
    out->ay = (float)ay_raw;
    out->az = (float)az_raw;
    
    // Read gyroscope data
    int16_t gx_raw, gy_raw, gz_raw;
    if (bno055_read16(port, addr, BNO055_GYRO_DATA_X_LSB_ADDR, &gx_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_GYRO_DATA_Y_LSB_ADDR, &gy_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_GYRO_DATA_Z_LSB_ADDR, &gz_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert to rad/s (LSB = 1/900 rad/s)
    out->gx = (float)gx_raw / 900.0f;
    out->gy = (float)gy_raw / 900.0f;
    out->gz = (float)gz_raw / 900.0f;
    
    // Read magnetometer data
    int16_t mx_raw, my_raw, mz_raw;
    if (bno055_read16(port, addr, BNO055_MAG_DATA_X_LSB_ADDR, &mx_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_MAG_DATA_Y_LSB_ADDR, &my_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_MAG_DATA_Z_LSB_ADDR, &mz_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert to μT (LSB = 1/16 μT)
    out->mx = (float)mx_raw / 16.0f;
    out->my = (float)my_raw / 16.0f;
    out->mz = (float)mz_raw / 16.0f;
    
    // Read Euler angles
    int16_t roll_raw, pitch_raw, yaw_raw;
    if (bno055_read16(port, addr, BNO055_EULER_R_LSB_ADDR, &roll_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_EULER_P_LSB_ADDR, &pitch_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_EULER_H_LSB_ADDR, &yaw_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert to degrees (LSB = 1/16 degrees)
    out->roll = (float)roll_raw / 16.0f;
    out->pitch = (float)pitch_raw / 16.0f;
    out->yaw = (float)yaw_raw / 16.0f;
    
    // Read quaternion
    int16_t qw_raw, qx_raw, qy_raw, qz_raw;
    if (bno055_read16(port, addr, BNO055_QUATERNION_DATA_W_LSB_ADDR, &qw_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_QUATERNION_DATA_X_LSB_ADDR, &qx_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_QUATERNION_DATA_Y_LSB_ADDR, &qy_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_QUATERNION_DATA_Z_LSB_ADDR, &qz_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert quaternion (LSB = 1/(2^14))
    out->qw = (float)qw_raw / 16384.0f;
    out->qx = (float)qx_raw / 16384.0f;
    out->qy = (float)qy_raw / 16384.0f;
    out->qz = (float)qz_raw / 16384.0f;
    
    // Read temperature
    uint8_t temp_raw;
    if (bno055_read8(port, addr, BNO055_TEMP_ADDR, &temp_raw) == ESP_OK) {
        out->temp = (float)temp_raw;
    } else {
        out->temp = 0.0f;
    }
    
    // Read calibration status
    bno055_get_calibration_status(port, addr, &out->sys_cal, &out->gyro_cal, &out->accel_cal, &out->mag_cal);
    
    return ESP_OK;
}
