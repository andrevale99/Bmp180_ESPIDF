#include <stdio.h>
#include <math.h>

#include "Bmp180.h"

#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

#define BMP180_ADDR 0x77

#define BMP180_START_MEASURMENT_REG 0xF4 //start measurment  register
#define BMP180_READ_ADC_MSB_REG 0xF6	 //read adc msb  register
#define BMP180_READ_ADC_LSB_REG 0xF7	 //read adc lsb  register
#define BMP180_READ_ADC_XLSB_REG 0xF8	 //read adc xlsb register

/* BMP180_START_MEASURMENT_REG controls */
#define BMP180_GET_TEMPERATURE_CTRL 0x2E   //get temperature control
#define BMP180_GET_PRESSURE_OSS0_CTRL 0x34 //get pressure oversampling 1 time/oss0 control
#define BMP180_GET_PRESSURE_OSS1_CTRL 0x74 //get pressure oversampling 2 time/oss1 control
#define BMP180_GET_PRESSURE_OSS2_CTRL 0xB4 //get pressure oversampling 4 time/oss2 control
#define BMP180_GET_PRESSURE_OSS3_CTRL 0xF4 //get pressure oversampling 8 time/oss3 control

#define ACKM 1
#define ACKS 0

//=============================VARIÁVEIS=====================================

uint8_t resolution = 0;
uint8_t regControl = 0;
float resolution_convTIME = 0;

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

enum MODES
{
	BMP180_ULTRALOWPOWER = 0x00, //low power             mode, oss0
	BMP180_STANDARD = 0x01,		 //standard              mode, oss1
	BMP180_HIGHRES = 0x02,		 //high resolution       mode, oss2
	BMP180_ULTRAHIGHRES = 0x03	 //ultra high resolution mode, oss3
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

	if (signal == ESP_OK)
	{
		get_calibration_data();
		return ESP_OK;
	}
	else
		return ESP_FAIL;
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
 * CASO N SEJA NENHUM DOS MODOS, SERÁ ESCOLHIDO O PADRÃO
 * 
 * @brief Configura o mode de operação do BMP180
*/
void set_mode_oversampling(uint8_t mode)
{
	switch (mode)
	{
	case BMP180_ULTRALOWPOWER:
	{
		regControl = BMP180_GET_PRESSURE_OSS0_CTRL;
		resolution = BMP180_ULTRALOWPOWER;
		resolution_convTIME = 5;
	}
	break;

	case BMP180_STANDARD:
	{
		regControl = BMP180_GET_PRESSURE_OSS1_CTRL;
		resolution = BMP180_STANDARD;
		resolution_convTIME = 8;
	}
	break;

	case BMP180_HIGHRES:
	{
		regControl = BMP180_GET_PRESSURE_OSS2_CTRL;
		resolution = BMP180_HIGHRES;
		resolution_convTIME = 14;
	}
	break;

	case BMP180_ULTRAHIGHRES:
	{
		regControl = BMP180_GET_PRESSURE_OSS3_CTRL;
		resolution = BMP180_ULTRAHIGHRES;
		resolution_convTIME = 26;
	}
	break;

	default:
	{
		printf("\'MODE\' NAO ENCONTRADO\nDEFININDO COMO: STANDARD (0X01)\n");
		regControl = BMP180_GET_PRESSURE_OSS1_CTRL;
		resolution = BMP180_STANDARD;
		resolution_convTIME = 8;
	}
	break;
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
 * @brief Requisita o valor de UT especificado no datasheet
 * 
 * @return UT
 * 
*/
int32_t bmp180_get_ut()
{
	uint8_t data[2];

	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_START_MEASURMENT_REG, ACKS);
	i2c_master_write_byte(cmd, BMP180_GET_TEMPERATURE_CTRL, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(5 / portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_READ_ADC_MSB_REG, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, ACKS);
	i2c_master_read_byte(cmd, &data[0], ACKS);
	i2c_master_read_byte(cmd, &data[1], ACKM);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	//printf("DATA: %X \t %X\n", data[0], data[1]);

	return ((data[0] << 8) | data[1]);
}

/**
 * @brief Requisita o valor de UP especificado no datasheet
 * 
 * @return UP
 * 
*/
int32_t bmp180_get_up()
{
	uint8_t data[3];

	i2c_cmd_handle_t cmd;
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_START_MEASURMENT_REG, ACKS);
	i2c_master_write_byte(cmd, regControl, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(resolution_convTIME / portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_READ_ADC_MSB_REG, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, ACKS);
	i2c_master_read_byte(cmd, &data[0], ACKS);
	i2c_master_read_byte(cmd, &data[1], ACKM);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, ACKS);
	i2c_master_write_byte(cmd, BMP180_READ_ADC_XLSB_REG, ACKS);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, ACKS);
	i2c_master_read_byte(cmd, &data[2], ACKM);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	//printf("DATA: %X \t %X\t %X\n", data[0], data[1], data[2]);
	//printf("0x%X\n", (((data[0] << 8) | (data[1]))<<8) | data[2] );

	uint32_t value = ((((data[0] << 8) | (data[1]))<<8) | data[2]);

	value >>= (8 - resolution);
	//printf("0x%X\n",  value >> (8 - resolution));
	//printf("VALUE: 0x%X\n\n",  value);

	return value;

}

/**
 * @brief Computa o valor de B5 
 * 
 * @return B5
*/
int32_t calcB5()
{
	int32_t UT = bmp180_get_ut();

	int32_t x1 = (UT - _calCoeff.bmpAC6) * _calCoeff.bmpAC5 / pow(2, 15);
	int32_t x2 = _calCoeff.bmpMC * pow(2, 11) / (x1 + _calCoeff.bmpMD);
	return (x1 + x2);
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
	float b5 = calcB5();

	return ((b5 + 8) / pow(2, 4) * 0.1);
}

float bmp180_get_pressure(){
	float pressure = 0;

	int32_t UP = bmp180_get_up();

	int32_t b6 = (int32_t)calcB5() - 4000;
	int32_t x1 = ((int32_t)_calCoeff.bmpB2 * (b6*b6 / pow(2, 12))) / pow(2, 11);
	int32_t x2 = (int32_t)_calCoeff.bmpAC2 * b6 / pow(2, 11);
	int32_t x3 = x1 + x2;
	int32_t b3 = ((((int32_t)_calCoeff.bmpAC1 * 4+x3)<< resolution) + 2) /4;
	
	x1 = (int32_t)_calCoeff.bmpAC3 * b6 / pow(2, 13);
	x2 = ((int32_t)_calCoeff.bmpB1 * (b6* b6/ pow(2, 12))) / pow(2, 16);
	x3 = ((x1 + x2)+2) / 4;

	uint32_t b4 = (uint32_t)_calCoeff.bmpAC4 * (x3 + 32768L) / pow(2, 15);
	uint32_t b7 = ((unsigned long)UP - b3) * (50000UL >> resolution);

	if   (b7 < 0x80000000) pressure = (b7 * 2) / b4;
    else                   pressure = (b7 / b4) * 2;

	x1 = pow((pressure / 256), 2);
  	x1 = (x1 * 3038L) / pow(2, 16);
  	x2 = (-7357L * pressure) / pow(2, 16);

	pressure = pressure + ((x1 + x2 + 3791L) / pow(2, 4));

	return pressure;
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
