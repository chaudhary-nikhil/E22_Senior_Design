#include "bno055.h"
#include "bus_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BNO055";

// Session start time - resets to 0 on each boot
static uint32_t session_start_time = 0;

// Helper function to read 8-bit register with retry for clock stretching
static esp_err_t bno055_read8(int port, uint8_t addr, uint8_t reg, uint8_t *data) {
    esp_err_t err;
    int retries = 3;
    
    do {
        err = bus_i2c_wrrd(port, addr, reg, data, 1, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) break;
        
        // If I2C error, wait and retry (clock stretching issue)
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for clock stretching
            retries--;
        } else {
            break; // Other errors don't retry
        }
    } while (retries > 0);
    
    return err;
}

// Helper function to write 8-bit register with retry for clock stretching
static esp_err_t bno055_write8(int port, uint8_t addr, uint8_t reg, uint8_t data) {
    esp_err_t err;
    int retries = 3;
    
    do {
        err = bus_i2c_wr8(port, addr, reg, data, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) break;
        
        // If I2C error, wait and retry (clock stretching issue)
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for clock stretching
            retries--;
        } else {
            break; // Other errors don't retry
        }
    } while (retries > 0);
    
    return err;
}

// Helper function to read 16-bit register with retry for clock stretching
static esp_err_t bno055_read16(int port, uint8_t addr, uint8_t reg, int16_t *data) {
    uint8_t buffer[2];
    esp_err_t err;
    int retries = 3;
    
    do {
        err = bus_i2c_wrrd(port, addr, reg, buffer, 2, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            *data = (int16_t)((buffer[1] << 8) | buffer[0]);
            break;
        }
        
        // If I2C error, wait and retry (clock stretching issue)
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for clock stretching
            retries--;
        } else {
            break; // Other errors don't retry
        }
    } while (retries > 0);
    
    return err;
}

// Helper function to write 16-bit register with retry for clock stretching
static esp_err_t bno055_write16(int port, uint8_t addr, uint8_t reg, int16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;                           // Register address
    buffer[1] = (uint8_t)(data & 0xFF);        // LSB
    buffer[2] = (uint8_t)((data >> 8) & 0xFF); // MSB
    
    esp_err_t err;
    int retries = 3;
    
    do {
        err = i2c_master_write_to_device(port, addr, buffer, 3, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) break;
        
        // If I2C error, wait and retry (clock stretching issue)
        if (err == ESP_ERR_TIMEOUT || err == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for clock stretching
            retries--;
        } else {
            break; // Other errors don't retry
        }
    } while (retries > 0);
    
    return err;
}

esp_err_t bno055_init(int port, uint8_t addr) {
    ESP_LOGI(TAG, "Initializing BNO055 at address 0x%02X", addr);
    
    // Wait for BNO055 to boot up
    vTaskDelay(pdMS_TO_TICKS(650));
    
    // Try both I2C addresses
    uint8_t addresses[] = {addr, (addr == BNO055_ADDR_A) ? BNO055_ADDR_B : BNO055_ADDR_A};
    uint8_t working_addr = 0;
    uint8_t chip_id;
    esp_err_t err = ESP_FAIL;
    
    for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
        uint8_t test_addr = addresses[addr_idx];
        
        int retry_count = 0;
        const int max_retries = 20;
        
        do {
            err = bno055_read8(port, test_addr, BNO055_CHIP_ID_ADDR, &chip_id);
            if (err == ESP_OK && chip_id == BNO055_ID) {
                working_addr = test_addr;
                ESP_LOGI(TAG, "BNO055 found at address 0x%02X", test_addr);
                break;
            }
            
            retry_count++;
            if (retry_count < max_retries) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        } while (retry_count < max_retries);
        
        if (err == ESP_OK && chip_id == BNO055_ID) {
            break;
        }
    }
    
    if (err != ESP_OK || chip_id != BNO055_ID) {
        ESP_LOGE(TAG, "Failed to find BNO055");
        return ESP_ERR_NOT_FOUND;
    }
    
    addr = working_addr;
    
    // Reset and configure
    ESP_LOGI(TAG, "Resetting BNO055...");
    err = bno055_reset(port, addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 reset failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Waiting for BNO055 to stabilize after reset...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Much longer delay after reset
    
    // Set to config mode
    ESP_LOGI(TAG, "Setting BNO055 to config mode...");
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 set config mode failed: %s", esp_err_to_name(err));
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for mode change
    
    // Configure device
    ESP_LOGI(TAG, "Configuring BNO055 power mode...");
    err = bno055_write8(port, addr, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 set power mode failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Setting BNO055 page ID...");
    err = bno055_write8(port, addr, BNO055_PAGE_ID_ADDR, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 set page ID failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Setting BNO055 unit selection...");
    err = bno055_write8(port, addr, BNO055_UNIT_SEL_ADDR, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 set unit selection failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set to NDOF mode
    ESP_LOGI(TAG, "Setting BNO055 to NDOF mode...");
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BNO055 set NDOF mode failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "Waiting for BNO055 fusion to start...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for fusion to start
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "BNO055 initialized successfully");
    return ESP_OK;
}

esp_err_t bno055_set_operation_mode(int port, uint8_t addr, bno055_opmode_t mode) {
    return bno055_write8(port, addr, BNO055_OPR_MODE_ADDR, (uint8_t)mode);
}

esp_err_t bno055_reset(int port, uint8_t addr) {
    // Write reset command to SYS_TRIGGER register
    return bno055_write8(port, addr, BNO055_SYS_TRIGGER_ADDR, 0x20);
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
    
    // Convert to m/s² (LSB = 1 mg = 0.001 m/s²)
    out->ax = (float)ax_raw / 1000.0f;
    out->ay = (float)ay_raw / 1000.0f;
    out->az = (float)az_raw / 1000.0f;
    
    // Read gyroscope data
    int16_t gx_raw, gy_raw, gz_raw;
    if (bno055_read16(port, addr, BNO055_GYRO_DATA_X_LSB_ADDR, &gx_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_GYRO_DATA_Y_LSB_ADDR, &gy_raw) != ESP_OK ||
        bno055_read16(port, addr, BNO055_GYRO_DATA_Z_LSB_ADDR, &gz_raw) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Convert to rad/s (LSB = 1/900 dps, convert dps to rad/s)
    out->gx = (float)gx_raw / 900.0f * (3.14159265359f / 180.0f);
    out->gy = (float)gy_raw / 900.0f * (3.14159265359f / 180.0f);
    out->gz = (float)gz_raw / 900.0f * (3.14159265359f / 180.0f);
    
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

// Write calibration offsets to BNO055 registers (as described in journal)
// This implements the hardware-side calibration approach
esp_err_t bno055_set_calibration_offsets(int port, uint8_t addr,
                                         int16_t accel_x, int16_t accel_y, int16_t accel_z,
                                         int16_t gyro_x, int16_t gyro_y, int16_t gyro_z,
                                         int16_t mag_x, int16_t mag_y, int16_t mag_z) {
    esp_err_t err;
    
    // Must be in config mode to write calibration offsets
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode for calibration");
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(25)); // Wait for mode change (datasheet says 19ms minimum)
    
    // Write accelerometer offsets
    err = bno055_write16(port, addr, BNO055_ACCEL_OFFSET_X_LSB_ADDR, accel_x);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_ACCEL_OFFSET_Y_LSB_ADDR, accel_y);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_ACCEL_OFFSET_Z_LSB_ADDR, accel_z);
    if (err != ESP_OK) return err;
    
    // Write gyroscope offsets
    err = bno055_write16(port, addr, BNO055_GYRO_OFFSET_X_LSB_ADDR, gyro_x);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_GYRO_OFFSET_Y_LSB_ADDR, gyro_y);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_GYRO_OFFSET_Z_LSB_ADDR, gyro_z);
    if (err != ESP_OK) return err;
    
    // Write magnetometer offsets
    err = bno055_write16(port, addr, BNO055_MAG_OFFSET_X_LSB_ADDR, mag_x);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_MAG_OFFSET_Y_LSB_ADDR, mag_y);
    if (err != ESP_OK) return err;
    err = bno055_write16(port, addr, BNO055_MAG_OFFSET_Z_LSB_ADDR, mag_z);
    if (err != ESP_OK) return err;
    
    // Switch back to NDOF fusion mode
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode after calibration");
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(25)); // Wait for mode change
    
    ESP_LOGI(TAG, "Calibration offsets written successfully");
    return ESP_OK;
}

// Configure axis remapping (as described in journal)
// This allows remapping physical axes to match sensor mounting orientation
esp_err_t bno055_set_axis_remap(int port, uint8_t addr, uint8_t axis_map_config, uint8_t axis_map_sign) {
    esp_err_t err;
    
    // Must be in config mode to write axis remap registers
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set config mode for axis remap");
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(25)); // Wait for mode change
    
    // Write axis remap configuration
    err = bno055_write8(port, addr, BNO055_AXIS_MAP_CONFIG_ADDR, axis_map_config);
    if (err != ESP_OK) return err;
    
    // Write axis remap sign (for inverting axes)
    err = bno055_write8(port, addr, BNO055_AXIS_MAP_SIGN_ADDR, axis_map_sign);
    if (err != ESP_OK) return err;
    
    // Switch back to NDOF fusion mode
    err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_NDOF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set NDOF mode after axis remap");
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(25)); // Wait for mode change
    
    ESP_LOGI(TAG, "Axis remapping configured successfully");
    return ESP_OK;
}

// Set up axis isolation for translation tracking (as described in journal)
// This calibrates Y and Z axes to stay fixed while tracking X-axis movement
esp_err_t bno055_setup_axis_isolation(int port, uint8_t addr, uint8_t track_axis) {
    esp_err_t err;
    
    // Read current sensor values to get baseline offsets
    bno055_sample_t sample;
    err = bno055_read_sample(port, addr, &sample);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sample for axis isolation setup");
        return err;
    }
    
    // Convert float values to raw offset values
    // For accelerometer: offset is in LSB units (1 LSB = 1 mg = 0.001 m/s²)
    // For gyroscope: offset is in LSB units (1 LSB = 1/900 dps)
    int16_t accel_x_offset = (int16_t)(sample.ax * 1000.0f);
    int16_t accel_y_offset = (int16_t)(sample.ay * 1000.0f);
    int16_t accel_z_offset = (int16_t)(sample.az * 1000.0f);
    
    int16_t gyro_x_offset = (int16_t)(sample.gx * 900.0f * 180.0f / 3.14159265359f);
    int16_t gyro_y_offset = (int16_t)(sample.gy * 900.0f * 180.0f / 3.14159265359f);
    int16_t gyro_z_offset = (int16_t)(sample.gz * 900.0f * 180.0f / 3.14159265359f);
    
    // If tracking X-axis, set Y and Z offsets to lock them
    // If tracking Y-axis, set X and Z offsets to lock them
    // If tracking Z-axis, set X and Y offsets to lock them
    if (track_axis == 0) {  // Track X, lock Y and Z
        accel_y_offset = (int16_t)(sample.ay * 1000.0f);
        accel_z_offset = (int16_t)(sample.az * 1000.0f);
        gyro_y_offset = (int16_t)(sample.gy * 900.0f * 180.0f / 3.14159265359f);
        gyro_z_offset = (int16_t)(sample.gz * 900.0f * 180.0f / 3.14159265359f);
        accel_x_offset = 0;  // Don't offset X-axis
        gyro_x_offset = 0;
    } else if (track_axis == 1) {  // Track Y, lock X and Z
        accel_x_offset = (int16_t)(sample.ax * 1000.0f);
        accel_z_offset = (int16_t)(sample.az * 1000.0f);
        gyro_x_offset = (int16_t)(sample.gx * 900.0f * 180.0f / 3.14159265359f);
        gyro_z_offset = (int16_t)(sample.gz * 900.0f * 180.0f / 3.14159265359f);
        accel_y_offset = 0;  // Don't offset Y-axis
        gyro_y_offset = 0;
    } else {  // Track Z, lock X and Y
        accel_x_offset = (int16_t)(sample.ax * 1000.0f);
        accel_y_offset = (int16_t)(sample.ay * 1000.0f);
        gyro_x_offset = (int16_t)(sample.gx * 900.0f * 180.0f / 3.14159265359f);
        gyro_y_offset = (int16_t)(sample.gy * 900.0f * 180.0f / 3.14159265359f);
        accel_z_offset = 0;  // Don't offset Z-axis
        gyro_z_offset = 0;
    }
    
    // Set magnetometer offsets to zero (we're not using magnetometer for translation)
    int16_t mag_x_offset = 0;
    int16_t mag_y_offset = 0;
    int16_t mag_z_offset = 0;
    
    // Write calibration offsets
    err = bno055_set_calibration_offsets(port, addr,
                                        accel_x_offset, accel_y_offset, accel_z_offset,
                                        gyro_x_offset, gyro_y_offset, gyro_z_offset,
                                        mag_x_offset, mag_y_offset, mag_z_offset);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Axis isolation configured: tracking axis %d", track_axis);
    }
    
    return err;
}
