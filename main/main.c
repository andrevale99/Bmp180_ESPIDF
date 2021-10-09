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
	
	esp_err_t signal_err = bmp180_begin(&conf, I2C_SCL, I2C_SDA);
	printf("%s ao inciar o BMP180\n\n", ((signal_err == ESP_OK)?"SUCESS":"FAIL")); 

	float T = bmp180_get_temperature();
	printf("T = %f °C\n", T);
	
	return;
}
