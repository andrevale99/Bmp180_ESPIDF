#include <stdio.h>
#include <math.h>

#include "Bmp180.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define BMP180_ADDR 0x77
#define BMP180_START_MEASURMENT_REG 0xF4 //start measurment  register
#define BMP180_TEM_REG 0x2E

#define ACKM 1
#define ACKS 0

//=============================VARIÁVEIS=====================================

enum COEFS
{
	BMP180_CAL_AC1_REG = 0xAA, //ac1 pressure    computation
	BMP180_CAL_AC2_REG = 0xAC, //ac2 pressure    computation
	BMP180_CAL_AC3_REG = 0xAE, //ac3 pressure    computation
	BMP180_CAL_AC4_REG = 0xB0, //ac4 pressure    computation
	BMP180_CAL_AC5_REG = 0xB2, //ac5 temperature computation
	BMP180_CAL_AC6_REG = 0xB4, //ac6 temperature computation
	BMP180_CAL_B1_REG = 0xB6,  //b1  pressure    computation
	BMP180_CAL_B2_REG = 0xB8,  //b2  pressure    computation
	BMP180_CAL_MB_REG = 0xBA,  //mb
	BMP180_CAL_MC_REG = 0xBC,  //mc  temperature computation
	BMP180_CAL_MD_REG = 0xBE   //md  temperature computation
};

struct BMP180_COEFS
{
	int16_t bmpAC1;
	int16_t bmpAC2;
	int16_t bmpAC3;
	uint16_t bmpAC4;
	uint16_t bmpAC5;
	uint16_t bmpAC6;
	int16_t bmpB1;
	int16_t bmpB2;
	int16_t bmpMB;
	int16_t bmpMC;
	int16_t bmpMD;
} _calCoeff;

//=====================================================================================

/**
 * @brief Configura a porta I2C com o ESP sendo MASTER
 * 
 * @param c --> Estrutura para a configuração do I2C 
 * @param scl --> Pino da porta SCL
 * @param sda --> Pino da porta SDA
 * 		  
 * @return retorna um valor inteiro para saber se deu certo ou não
*/

esp_err_t bmp180_begin(i2c_config_t *c, uint8_t pin_scl, uint8_t pin_sda)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = pin_sda, // select GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = pin_scl, // select GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ, // select frequency specific to your project
	};

	i2c_param_config(I2C_MASTER_NUM, &conf);

	esp_err_t signal = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

	if(signal == ESP_OK){
		get_calibration_data();
		return ESP_OK;
	}
	else return ESP_FAIL;
}

/**
 * @brief Requisita os dados de calibração do sensor para realizar os cálculos
*/
void get_calibration_data()
{

	uint32_t value = 0;

	for (uint8_t reg = BMP180_CAL_AC1_REG; reg <= BMP180_CAL_MD_REG; ++reg)
	{
		value = read16_bits(reg);

		switch (reg)
		{
		case BMP180_CAL_AC1_REG: //used for pressure computation
			_calCoeff.bmpAC1 = value;
			break;

		case BMP180_CAL_AC2_REG: //used for pressure computation
			_calCoeff.bmpAC2 = value;
			break;

		case BMP180_CAL_AC3_REG: //used for pressure computation
			_calCoeff.bmpAC3 = value;
			break;

		case BMP180_CAL_AC4_REG: //used for pressure computation
			_calCoeff.bmpAC4 = value;
			break;

		case BMP180_CAL_AC5_REG: //used for temperature computation
			_calCoeff.bmpAC5 = value;
			break;

		case BMP180_CAL_AC6_REG: //used for temperature computation
			_calCoeff.bmpAC6 = value;
			break;

		case BMP180_CAL_B1_REG: //used for pressure computation
			_calCoeff.bmpB1 = value;
			break;

		case BMP180_CAL_B2_REG: //used for pressure computation
			_calCoeff.bmpB2 = value;
			break;

		case BMP180_CAL_MB_REG: //???
			_calCoeff.bmpMB = value;
			break;

		case BMP180_CAL_MC_REG: //used for temperature computation
			_calCoeff.bmpMC = value;
			break;

		case BMP180_CAL_MD_REG: //used for temperature computation
			_calCoeff.bmpMD = value;
			break;
		}
	}
}

/**
 * @brief função debug para visualizar os dados dos coeficientes de calibração
*/
void print_calibration_data()
{
	printf("***************************************\n");
	printf("AC1: %i\n", _calCoeff.bmpAC1);
	printf("AC2: %i\n", _calCoeff.bmpAC2);
	printf("AC3: %i\n", _calCoeff.bmpAC3);
	printf("AC4: %i\n", _calCoeff.bmpAC4);
	printf("AC5: %i\n", _calCoeff.bmpAC5);
	printf("AC6: %i\n", _calCoeff.bmpAC6);
	printf("B1: %i\n", _calCoeff.bmpB1);
	printf("B2: %i\n", _calCoeff.bmpB2);
	printf("MB: %i\n", _calCoeff.bmpMB);
	printf("MC: %i\n", _calCoeff.bmpMC);
	printf("MD: %i\n", _calCoeff.bmpMD);
	printf("***************************************\n");

	fflush(stdout);
}

/**
 * @brief Requisita os 2 bytes da temperatura(função de debug)
 * 
 * @param data --> Variável onde será armazenado os bytes
 * 
*/
uint32_t bmp180_get_ut()
{
	uint8_t data[2];

	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_START_MEASURMENT_REG, ACKS);
	i2c_master_write_byte(cmd, BMP180_TEM_REG, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(5 / portTICK_RATE_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, 0xF6, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, ACKS);

	i2c_master_read_byte(cmd, &data[0], ACKM);
	i2c_master_read_byte(cmd, &data[1], ACKM);

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ((data[0] << 8) | data[1]);
}

/**
 * A função retorna em "float", porém dependendo do microcontrolador
 * pode modificar a função para retornar um long ou outro tipo primitivo de
 * 4 bytes, como especificado no datasheet
 * 
 * @brief função para calcular a temperatura
 * 
 * @return Temperatura em Celsius
*/
float bmp180_get_temperature()
{
	uint32_t UT = bmp180_get_ut();

	float x1 = (UT - _calCoeff.bmpAC6) * _calCoeff.bmpAC5 / pow(2, 15);
	float x2 = _calCoeff.bmpMC * pow(2, 11) / (x1 + _calCoeff.bmpMD);
	float b5 = x1 + x2;

	return ((b5 + 8) / pow(2, 4) * 0.1);
}

//==========================================================================================================================
/**
 * Algumas funções que criei e pensei que poderia utilizar, por causa da biblioteca de referência
*/

/**
 * @brief Ler 16 bytes a partir da requisição do registrador
 * 
 * @param reg --> Registrador do BMP180
 * 
 * @return os 16 bits queforam requisitados
*/
uint16_t read16_bits(uint16_t reg)
{
	uint8_t data[2];

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, reg, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, ACKS);
	i2c_master_read(cmd, data, 2, ACKM);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return ((data[0] << 8) | data[1]);
}

/**
 * @brief Escreve 1 byte no sensor
 * 
 * @param reg --> Registrador de Comando (exp.: BMP180_START_MEASURMENT_REG 0xF4)
 * @param control --> Registrador do que deseja (exp.: BMP180_TEM_REG 0X2E)
 * 
 * @return retorna o sinal de erro ou sucesso (Ver documentação da variavel "esp_err_t" na ESP-IDF)
*/
esp_err_t write8_bits(uint8_t reg, uint8_t control)
{

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKM);
	i2c_master_write_byte(cmd, reg, ACKM);
	i2c_master_write_byte(cmd, control, ACKS);
	i2c_master_stop(cmd);
	esp_err_t signal = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return signal;
}
