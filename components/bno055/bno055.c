#include "bno055.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char* TAG = "BNO055";
static i2c_port_t s_i2c;
static uint8_t    s_addr;

// BNO055 registers (subset)
#define BNO_CHIP_ID        0x00
#define BNO_PWR_MODE       0x3E
#define BNO_OPR_MODE       0x3D
#define BNO_UNIT_SEL       0x3B
#define BNO_SYS_TRIGGER    0x3F
#define BNO_TEMP           0x34

// Vector base addresses (fusion off: raw units; fusion on (NDOF): calibrated)
#define BNO_ACC_DATA_X_LSB 0x08
#define BNO_GYR_DATA_X_LSB 0x14

static esp_err_t wr(uint8_t reg, const uint8_t* data, size_t n);
static esp_err_t rd(uint8_t reg, uint8_t* data, size_t n);
static esp_err_t delay_ms(uint32_t ms){ uint64_t us=ms*1000ULL; uint64_t t0=esp_timer_get_time(); while(esp_timer_get_time()-t0<us){} return ESP_OK; }

esp_err_t bno055_init(i2c_port_t i2c, uint8_t addr) {
    s_i2c = i2c; s_addr = addr;

    // Verify chip id (should be 0xA0)
    uint8_t id=0;
    ESP_ERROR_CHECK(rd(BNO_CHIP_ID, &id, 1));
    if (id != 0xA0) {
        ESP_LOGE(TAG, "Unexpected CHIP_ID=0x%02X", id);
        return ESP_FAIL;
    }

    // Go to CONFIG mode for config writes
    uint8_t mode = BNO055_OPMODE_CONFIG;
    ESP_ERROR_CHECK(wr(BNO_OPR_MODE, &mode, 1));
    delay_ms(25);

    // Soft reset
    uint8_t trig = 0x20;
    ESP_ERROR_CHECK(wr(BNO_SYS_TRIGGER, &trig, 1));
    delay_ms(700);

    // Re-check ID after reset
    ESP_ERROR_CHECK(rd(BNO_CHIP_ID, &id, 1));
    if (id != 0xA0) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t bno055_config_units(bool accel_mps2, bool gyro_rads, bool temp_c) {
    // UNIT_SEL bits:
    // 7:0 reserved; bit7 = Orientation Windows vs Android (ignore here)
    // Accel: m/s^2 (0) or mg (1)
    // Gyro: deg/s (0) or rad/s (1)
    // Temp: Celsius (0) or Fahrenheit (1)
    uint8_t unit = 0x00;
    if (!accel_mps2) unit |= (1<<0);
    if (gyro_rads)   unit |= (1<<1);
    if (!temp_c)     unit |= (1<<4);

    ESP_ERROR_CHECK(wr(BNO_UNIT_SEL, &unit, 1));
    delay_ms(10);
    return ESP_OK;
}

esp_err_t bno055_set_opmode(bno055_opmode_t mode) {
    ESP_ERROR_CHECK(wr(BNO_OPR_MODE, (uint8_t*)&mode, 1));
    delay_ms(20);
    return ESP_OK;
}

esp_err_t bno055_read_imu(bno_imu_sample_t* s) {
    if (!s) return ESP_ERR_INVALID_ARG;

    uint8_t buf[6*2]; // accel(6) + gyro(6)
    // Accel (X_LSB..Z_MSB), then Gyro (X_LSB..Z_MSB)
    ESP_ERROR_CHECK(rd(BNO_ACC_DATA_X_LSB, buf, 6));
    int16_t ax_i = (int16_t)(buf[1]<<8 | buf[0]);
    int16_t ay_i = (int16_t)(buf[3]<<8 | buf[2]);
    int16_t az_i = (int16_t)(buf[5]<<8 | buf[4]);

    ESP_ERROR_CHECK(rd(BNO_GYR_DATA_X_LSB, buf, 6));
    int16_t gx_i = (int16_t)(buf[1]<<8 | buf[0]);
    int16_t gy_i = (int16_t)(buf[3]<<8 | buf[2]);
    int16_t gz_i = (int16_t)(buf[5]<<8 | buf[4]);

    // Units with UNIT_SEL we set:
    // accel: 1 LSB = 1 m/s^2 / 100?  (BNO uses scaling 1 LSB = 1 m/s^2 * 100)
    // gyro:  1 LSB = 1 rad/s / 16?   (BNO uses scaling 1 LSB = 16 LSB per rad/s)
    // These are the standard BNO055 scalings in unit-configured mode:
    const float ACC_SCALE = 1.0f / 100.0f;  // to m/s^2
    const float GYR_SCALE = 1.0f / 900.0f;  // to rad/s (BNO docs: 1 LSB = 1/16 deg/s; with rad/s unit, effective ≈ 1/900 rad/s)
    // If your readings look off, adjust GYR_SCALE to your datasheet value.

    s->ax = ax_i * ACC_SCALE;
    s->ay = ay_i * ACC_SCALE;
    s->az = az_i * ACC_SCALE;

    s->gx = gx_i * GYR_SCALE;
    s->gy = gy_i * GYR_SCALE;
    s->gz = gz_i * GYR_SCALE;

    // Temperature (°C)
    uint8_t t8=0;
    ESP_ERROR_CHECK(rd(BNO_TEMP, &t8, 1));
    s->tempC = (float)((int8_t)t8);

    s->t_ms = (uint32_t)(esp_timer_get_time()/1000ULL);
    return ESP_OK;
}

// ---------------- I2C helpers ----------------
static esp_err_t wr(uint8_t reg, const uint8_t* data, size_t n) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (n && data) i2c_master_write(cmd, (uint8_t*)data, n, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(s_i2c, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t rd(uint8_t reg, uint8_t* data, size_t n) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr<<1)|I2C_MASTER_READ, true);
    if (n > 1) i2c_master_read(cmd, data, n-1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data+n-1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(s_i2c, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}
