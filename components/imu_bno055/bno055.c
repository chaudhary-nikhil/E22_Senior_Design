#include "bno055.h"
#include "bus_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "esp_rom_sys.h"

static const char *TAG = "BNO055";

// Session start time - resets to 0 on each boot
static uint32_t session_start_time = 0;

/* Updated only inside bno055_read_sample (sampling task). HTTP uses this instead of a concurrent I2C cal read. */
static volatile uint8_t s_last_cal_sys = 0;
static volatile uint8_t s_last_cal_gyro = 0;
static volatile uint8_t s_last_cal_accel = 0;
static volatile uint8_t s_last_cal_mag = 0;
/* Use NDOF for best orientation + gravity separation when available. */
static const bno055_opmode_t GF_TARGET_OPMODE = BNO055_OPERATION_MODE_NDOF;
static volatile uint8_t s_last_opmode = (uint8_t)BNO055_OPERATION_MODE_NDOF;

void bno055_get_last_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel,
                                 uint8_t *mag) {
  if (!sys || !gyro || !accel || !mag) {
    return;
  }
  *sys = s_last_cal_sys;
  *gyro = s_last_cal_gyro;
  *accel = s_last_cal_accel;
  *mag = s_last_cal_mag;
}

uint8_t bno055_get_last_opmode(void) { return s_last_opmode; }

// Helper function to read 8-bit register with retry for clock stretching
static esp_err_t bno055_read8(int port, uint8_t addr, uint8_t reg,
                              uint8_t *data) {
  esp_err_t err;
  int retries = 3;

  do {
    err = bus_i2c_wrrd(port, addr, reg, data, 1, pdMS_TO_TICKS(1000));
    if (err == ESP_OK)
      break;

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
static esp_err_t bno055_write8(int port, uint8_t addr, uint8_t reg,
                               uint8_t data) {
  esp_err_t err;
  int retries = 3;

  do {
    err = bus_i2c_wr8(port, addr, reg, data, pdMS_TO_TICKS(1000));
    if (err == ESP_OK)
      break;

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
static esp_err_t bno055_read16(int port, uint8_t addr, uint8_t reg,
                               int16_t *data) {
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

#define BNO055_NRESET_GPIO  17   // whatever pin you flywired to

static void bno055_hw_reset(void) {
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << BNO055_NRESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&rst_cfg);

    gpio_set_level(BNO055_NRESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Drive reset low for 10ms (datasheet: min 20ns, use 10ms for margin)
    gpio_set_level(BNO055_NRESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Release reset — R8 and our GPIO will both drive high
    gpio_set_level(BNO055_NRESET_GPIO, 1);

    ESP_LOGI(TAG, "BNO055 hardware reset toggled on GPIO%d", BNO055_NRESET_GPIO);
}

esp_err_t bno055_reset(int port, uint8_t addr) {
  // Write reset command to SYS_TRIGGER register.
  // NOTE: This write may NACK because the chip resets mid-transaction.
  // We ignore the return — the reset happens regardless.
  (void)bno055_write8(port, addr, BNO055_SYS_TRIGGER_ADDR, 0x20);
  return ESP_OK;
}

esp_err_t bno055_set_operation_mode(int port, uint8_t addr,
                                    bno055_opmode_t mode) {
  esp_err_t err = bno055_write8(port, addr, BNO055_OPR_MODE_ADDR, (uint8_t)mode);
  if (err == ESP_OK) {
    s_last_opmode = (uint8_t)mode;
    // Datasheet table 3-6: any→CONFIG needs 7ms, CONFIG→fusion needs 19ms.
    // Use 30ms to cover both cases with margin.
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  return err;
}

esp_err_t bno055_init(int port, uint8_t addr) {
  ESP_LOGI(TAG, "Initializing BNO055 at address 0x%02X", addr);

  // Wait for BNO055 to boot up after power-on
  // vTaskDelay(pdMS_TO_TICKS(650));

  // Hardware reset via flywired GPIO → nRESET
  bno055_hw_reset();
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Try both I2C addresses
  uint8_t addresses[] = {addr, (addr == BNO055_ADDR_A) ? BNO055_ADDR_B
                                                       : BNO055_ADDR_A};
  uint8_t working_addr = 0;
  uint8_t chip_id = 0;
  esp_err_t err = ESP_FAIL;

  for (int addr_idx = 0; addr_idx < 2; addr_idx++) {
    uint8_t test_addr = addresses[addr_idx];

    int retry_count = 0;
    const int max_retries = 3;

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

  // // Reset
  // ESP_LOGI(TAG, "Resetting BNO055...");
  // err = bno055_reset(port, addr);
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "BNO055 reset failed: %s", esp_err_to_name(err));
  //   return err;
  // }

  // After soft reset, chip reboots and drops off the I2C bus for ~650ms.
  // Boot time varies chip-to-chip; poll CHIP_ID until it responds rather
  // than waiting a fixed time.
  // ESP_LOGI(TAG, "Waiting for BNO055 to come back after reset...");
  // const int reset_timeout_ms = 3000;
  // const int poll_interval_ms = 20;
  // int elapsed_ms = 650;
  // bool chip_back = false;
  // // Initial grace period — chip is guaranteed unresponsive for the first ~650ms
  // vTaskDelay(pdMS_TO_TICKS(650));
  // while (elapsed_ms < reset_timeout_ms) {
  //   uint8_t id = 0;
  //   esp_err_t poll_err = bno055_read8(port, addr, BNO055_CHIP_ID_ADDR, &id);
  //   if (poll_err == ESP_OK && id == BNO055_ID) {
  //     chip_back = true;
  //     ESP_LOGI(TAG, "BNO055 responsive after reset (~%d ms)", elapsed_ms);
  //     break;
  //   }
  //   vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
  //   elapsed_ms += poll_interval_ms;
  // }
  // if (!chip_back) {
  //   ESP_LOGE(TAG, "BNO055 did not come back after reset within %d ms",
  //            reset_timeout_ms);
  //   return ESP_ERR_TIMEOUT;
  // }

  // Set to config mode
  ESP_LOGI(TAG, "Setting BNO055 to config mode...");
  err = bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BNO055 set config mode failed: %s", esp_err_to_name(err));
    return err;
  } 

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

  // Set fusion operation mode (NDOF for full 9-DOF orientation when calibrated)
  ESP_LOGI(TAG, "Setting BNO055 to fusion mode 0x%02X...", (unsigned)GF_TARGET_OPMODE);
  err = bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "BNO055 set fusion mode failed: %s", esp_err_to_name(err));
    return err;
  }

  // Wait for fusion algorithm to produce valid output
  ESP_LOGI(TAG, "Waiting for BNO055 fusion to start...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  ESP_LOGI(TAG, "BNO055 initialized successfully in mode 0x%02X", (unsigned)GF_TARGET_OPMODE);
  return ESP_OK;
}

esp_err_t bno055_get_calibration_status(int port, uint8_t addr, uint8_t *sys,
                                        uint8_t *gyro, uint8_t *accel,
                                        uint8_t *mag) {
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
  if (bno055_read16(port, addr, BNO055_ACCEL_DATA_X_LSB_ADDR, &ax_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_ACCEL_DATA_Y_LSB_ADDR, &ay_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_ACCEL_DATA_Z_LSB_ADDR, &az_raw) !=
          ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert to m/s² (LSB = 1 mg = 0.001 m/s²)
  out->ax = (float)ax_raw / 100.0f;
  out->ay = (float)ay_raw / 100.0f;
  out->az = (float)az_raw / 100.0f;

  // Read gyroscope data
  int16_t gx_raw, gy_raw, gz_raw;
  if (bno055_read16(port, addr, BNO055_GYRO_DATA_X_LSB_ADDR, &gx_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_GYRO_DATA_Y_LSB_ADDR, &gy_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_GYRO_DATA_Z_LSB_ADDR, &gz_raw) !=
          ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert to rad/s (LSB = 1/900 dps, convert dps to rad/s)
  out->gx = (float)gx_raw / 16.0f * (3.14159265359f / 180.0f);
  out->gy = (float)gy_raw / 16.0f * (3.14159265359f / 180.0f);
  out->gz = (float)gz_raw / 16.0f * (3.14159265359f / 180.0f);

  // Read magnetometer data
  int16_t mx_raw, my_raw, mz_raw;
  if (bno055_read16(port, addr, BNO055_MAG_DATA_X_LSB_ADDR, &mx_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_MAG_DATA_Y_LSB_ADDR, &my_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_MAG_DATA_Z_LSB_ADDR, &mz_raw) !=
          ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert to μT (LSB = 1/16 μT)
  out->mx = (float)mx_raw / 16.0f;
  out->my = (float)my_raw / 16.0f;
  out->mz = (float)mz_raw / 16.0f;

  // Read Euler angles
  int16_t roll_raw, pitch_raw, yaw_raw;
  if (bno055_read16(port, addr, BNO055_EULER_R_LSB_ADDR, &roll_raw) != ESP_OK ||
      bno055_read16(port, addr, BNO055_EULER_P_LSB_ADDR, &pitch_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_EULER_H_LSB_ADDR, &yaw_raw) != ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert to degrees (LSB = 1/16 degrees)
  out->roll = (float)roll_raw / 16.0f;
  out->pitch = (float)pitch_raw / 16.0f;
  out->yaw = (float)yaw_raw / 16.0f;

  // Read quaternion
  int16_t qw_raw, qx_raw, qy_raw, qz_raw;
  if (bno055_read16(port, addr, BNO055_QUATERNION_DATA_W_LSB_ADDR, &qw_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_QUATERNION_DATA_X_LSB_ADDR, &qx_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_QUATERNION_DATA_Y_LSB_ADDR, &qy_raw) !=
          ESP_OK ||
      bno055_read16(port, addr, BNO055_QUATERNION_DATA_Z_LSB_ADDR, &qz_raw) !=
          ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert quaternion (LSB = 1/(2^14))
  out->qw = (float)qw_raw / 16384.0f;
  out->qx = (float)qx_raw / 16384.0f;
  out->qy = (float)qy_raw / 16384.0f;
  out->qz = (float)qz_raw / 16384.0f;

  // Read Linear Acceleration data (Registers 0x28-0x2D)
  int16_t lia_x_raw, lia_y_raw, lia_z_raw;
  if (bno055_read16(port, addr, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
                    &lia_x_raw) != ESP_OK ||
      bno055_read16(port, addr, BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR,
                    &lia_y_raw) != ESP_OK ||
      bno055_read16(port, addr, BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR,
                    &lia_z_raw) != ESP_OK) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Convert to m/s^2 (LSB = 1/100 m/s^2)
  // This scaling comes from the Bosch datasheet/header
  // (BNO055_LINEAR_ACCEL_DIV_MSQ)
  out->lia_x = (float)lia_x_raw / 100.0f;
  out->lia_y = (float)lia_y_raw / 100.0f;
  out->lia_z = (float)lia_z_raw / 100.0f;

  // Read temperature
  uint8_t temp_raw;
  if (bno055_read8(port, addr, BNO055_TEMP_ADDR, &temp_raw) == ESP_OK) {
    out->temp = (float)temp_raw;
  } else {
    out->temp = 0.0f;
  }

  // Read calibration status
  bno055_get_calibration_status(port, addr, &out->sys_cal, &out->gyro_cal,
                                &out->accel_cal, &out->mag_cal);
  s_last_cal_sys = out->sys_cal;
  s_last_cal_gyro = out->gyro_cal;
  s_last_cal_accel = out->accel_cal;
  s_last_cal_mag = out->mag_cal;

  // Heartbeat log for mode/status debugging (every 500 samples ~5s)
  static uint32_t hb_count = 0;
  if (++hb_count % 500 == 0) {
    uint8_t mode = 0;
    bno055_read8(port, addr, BNO055_OPR_MODE_ADDR, &mode);
    ESP_LOGI(TAG, "Status: Mode=0x%02X, Cal: S%d G%d A%d M%d", 
             mode, out->sys_cal, out->gyro_cal, out->accel_cal, out->mag_cal);
  }

  return ESP_OK;
}

esp_err_t bno055_start_calibration(int port, uint8_t addr) {
  // BNO055 auto-calibrates in fusion modes; ensure we're in the target fusion mode.
  ESP_LOGI(TAG, "Starting BNO055 calibration in fusion mode 0x%02X", (unsigned)GF_TARGET_OPMODE);
  ESP_LOGI(TAG, "  Gyro: Leave device still for 3-5 seconds");
  ESP_LOGI(TAG, "  Accel: Slowly rotate device through 6 orientations");
  ESP_LOGI(TAG, "  Mag: Wide figure-8 away from metal (needed for NDOF)");
  return bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
}

esp_err_t bno055_save_calibration_data(int port, uint8_t addr,
                                       uint8_t *cal_data) {
  if (!cal_data)
    return ESP_ERR_INVALID_ARG;

  // Must be in CONFIG mode to read offset registers (BNO055 datasheet 3.6.4)
  esp_err_t err =
      bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
  if (err != ESP_OK)
    return err;
  vTaskDelay(pdMS_TO_TICKS(25));

  // Read 22 bytes: accel offsets (6) + mag offsets (6) + gyro offsets (6) +
  //                accel radius (2) + mag radius (2)
  for (int i = 0; i < 22; i++) {
    err = bno055_read8(port, addr, BNO055_ACCEL_OFFSET_X_LSB_ADDR + i,
                       &cal_data[i]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to read cal offset at reg 0x%02X",
               BNO055_ACCEL_OFFSET_X_LSB_ADDR + i);
      // Restore fusion mode before returning error
      bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
      return err;
    }
  }

  // Restore fusion mode
  err = bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "Calibration data saved (22 bytes)");
  return err;
}

esp_err_t bno055_load_calibration_data(int port, uint8_t addr,
                                       const uint8_t *cal_data) {
  if (!cal_data)
    return ESP_ERR_INVALID_ARG;

  // Must be in CONFIG mode to write offset registers
  esp_err_t err =
      bno055_set_operation_mode(port, addr, BNO055_OPERATION_MODE_CONFIG);
  if (err != ESP_OK)
    return err;
  vTaskDelay(pdMS_TO_TICKS(25));

  // Write 22 bytes of calibration offsets + radii
  for (int i = 0; i < 22; i++) {
    err = bno055_write8(port, addr, BNO055_ACCEL_OFFSET_X_LSB_ADDR + i,
                        cal_data[i]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to write cal offset at reg 0x%02X",
               BNO055_ACCEL_OFFSET_X_LSB_ADDR + i);
      bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
      return err;
    }
  }

  // Restore fusion mode
  err = bno055_set_operation_mode(port, addr, GF_TARGET_OPMODE);
  vTaskDelay(pdMS_TO_TICKS(20));

  ESP_LOGI(TAG, "Calibration data restored (22 bytes)");
  return err;
}
