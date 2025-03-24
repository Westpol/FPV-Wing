/*
 * bmp390.h
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 */

#ifndef INC_BMP390_H_
#define INC_BMP390_H_

#include "stm32f7xx_hal.h"

typedef struct {
	double pressure;
} Baro_Data;

extern Baro_Data baro_data;

int BMP_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN);
void BMP_READ_DATA();
void BMP_CALCULATE_VALUES(uint32_t time_us);

double BMP_GET_HEIGHT();
double BMP_GET_PRESS();
double BMP_GET_TEMP();

void BMI_GYRO_SOFT_RESET();

#define WRITE_BYTE 0x7F
#define READ_BYTE 0x80


#endif /* INC_BMP390_H_ */
