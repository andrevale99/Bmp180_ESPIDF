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
	
	esp_err_t signal_err = setup_i2c_master(&conf, I2C_SCL, I2C_SDA);
	printf("%s on install I2C\n\n", ((signal_err == ESP_OK)?"SUCESS":"FAIL")); 

	get_calibration_data();
	//print_calibration_data();

	uint16_t UT = bmp180_get_ut();

	printf("UT: %i\n", UT);

	fflush(stdout);
	
	return;
}
