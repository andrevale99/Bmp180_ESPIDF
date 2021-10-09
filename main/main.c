#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

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

	time_t now;
	struct tm *data;

	time(&now);
	data = localtime(&now);

	i2c_config_t conf;
	
	esp_err_t signal_err = bmp180_begin(&conf, I2C_SCL, I2C_SDA);
	printf("%s ao inciar o BMP180\n\n", ((signal_err == ESP_OK)?"SUCESS":"FAIL")); 

	float T = bmp180_get_temperature();

	while(true){
		T = bmp180_get_temperature();
		
		printf("%s", asctime(data));
		printf("T = %f °C\n\n", T);
		vTaskDelay( 1000 / portTICK_PERIOD_MS);

		time(&now);
		data = localtime(&now);
	}
	
	return;
}
