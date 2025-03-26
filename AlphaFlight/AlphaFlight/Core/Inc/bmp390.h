/*
 * bmp390.h
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include "stm32f7xx_hal.h"
#include "math.h"

typedef struct {
	double pressure;
	double temperature;
} Baro_Data;

typedef struct {
	uint16_t NVM_PAR_T1;
	uint16_t NVM_PAR_T2;
	int8_t NVM_PAR_T3;
	int16_t NVM_PAR_P1;
	int16_t NVM_PAR_P2;
	int8_t NVM_PAR_P3;
	int8_t NVM_PAR_P4;
	uint16_t NVM_PAR_P5;
	uint16_t NVM_PAR_P6;
	int8_t NVM_PAR_P7;
	int8_t NVM_PAR_P8;
	int16_t NVM_PAR_P9;
	int8_t NVM_PAR_P10;
	int8_t NVM_PAR_P11;
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} Baro_Calibration_Float;

int BMP_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *BARO_GPIOx, uint16_t BARO_PIN);

double BMP_GET_HEIGHT(double ground_pressure_pa);
double BMP_GET_PRESS();
double BMP_GET_TEMP();

void BMP_GET_DATA(void);

void BMP_SOFT_RESET(void);

//---------------------------------------- register defines -------------------------------------------------
#define WRITE_BYTE 0x7F
#define READ_BYTE 0x80

#define PRESSURE_DATA_REGISTER 0x04
#define TEMPERATURE_DATA_REGISTER 0x07

#define POWER_CONTROL_REGISTER 0x1B
#define POWER_CONTROL_PRESS_EN_TEMP_EN_NORMAL_MODE 0b00110011

#define OVERSAMPLING_REGISTER 0x1C
#define OSR_P_16X_T_2X 0b00001100

#define OUTPUT_DATA_RATE_REGISTER 0x1D
#define OUTPUT_DATA_RATE_25_HZ 0x03

#define IIR_FILTER_REGISTER 0x1F
#define IIR_FILTER_COEF_3 0b00000100
//-----------------------------------------------------------------------------------------------------------


#endif /* INC_BMP390_H_ */
