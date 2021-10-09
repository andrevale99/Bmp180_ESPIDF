#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"

#include "Bmp180.h"

#define I2C_SCL 22
#define I2C_SDA 21

void app_main(void)
{

	i2c_config_t conf;
	
	esp_err_t teste = setup_i2c_master(&conf, I2C_SCL, I2C_SDA);

	get_calibration_data();
	print_calibration_data();
	
	uint16_t value = 0;
	uint8_t data[2];

	get_temp_bytes(data);

	printf("0x%X \t 0x%X\n", data[0], data[1]);
	printf("%i\n", (data[0] << 8) | data[1]);


	return;
}
