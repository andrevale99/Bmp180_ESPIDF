#ifndef BMP180_ESPIDF_H
#define BMP180_ESPIDF_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_err.h"

int setup_i2c_master(i2c_config_t *c, uint8_t pin_scl, uint8_t pin_sda);
void get_temp_bytes(uint8_t *data);

void get_calibration_data();
void print_calibration_data();

uint16_t read16_bits(uint16_t reg);
esp_err_t write8_bits(uint8_t reg, uint8_t control);


#endif