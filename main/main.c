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

//Portas que serãoutilizadas pelo Protocolo I2C
#define I2C_SCL 22
#define I2C_SDA 21

/*
	BMP180_ULTRALOWPOWER = 0x00, //low power             mode, oss0
	BMP180_STANDARD = 0x01,		 //standard              mode, oss1
	BMP180_HIGHRES = 0x02,		 //high resolution       mode, oss2
	BMP180_ULTRAHIGHRES = 0x03	 //ultra high resolution mode, oss3
*/

void app_main(void)
{

	i2c_config_t conf;
	
	esp_err_t signal_err = bmp180_begin(&conf, I2C_SCL, I2C_SDA);
	printf("%s ao inciar o BMP180\n\n", ((signal_err == ESP_OK)?"SUCESS":"FAIL")); 

	set_mode_oversampling(0x03);

	float T = 0;
	float P = 0;

	while(true){
		T = bmp180_get_temperature();
		P = bmp180_get_pressure();

		printf("T = %f °C\n", T);
		printf("P = %f Pa\n\n", P);
		
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	
		
	return;
}
