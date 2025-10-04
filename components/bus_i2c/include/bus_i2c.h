#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bus_i2c_init(i2c_port_t port, int sda, int scl, uint32_t hz);
esp_err_t bus_i2c_wr8(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val, TickType_t to);
esp_err_t bus_i2c_wrrd(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, TickType_t to);

#ifdef __cplusplus
}
#endif
