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

#define GYRO_RANGE_ADRESS 0x0F
#define GYRO_RANGE_2000_DEG_PER_SECOND 0x00
#define GYRO_RANGE_1000_DEG_PER_SECOND 0x01
#define GYRO_RANGE_500_DEG_PER_SECOND 0x02
#define GYRO_RANGE_250_DEG_PER_SECOND 0x03
#define GYRO_RANGE_125_DEG_PER_SECOND 0x04

#define GYRO_ODR_FILTER_ADRESS 0x10
#define GYRO_ODR_2000_HZ_FILTER_532_HZ 0x00
#define GYRO_ODR_2000_HZ_FILTER_230_HZ 0x01
#define GYRO_ODR_1000_HZ_FILTER_116_HZ 0x02
#define GYRO_ODR_400_HZ_FILTER_47_HZ 0x03
#define GYRO_ODR_200_HZ_FILTER_23_HZ 0x04
#define GYRO_ODR_100_HZ_FILTER_12_HZ 0x05
#define GYRO_ODR_200_HZ_FILTER_64_HZ 0x06
#define GYRO_ODR_100_HZ_FILTER_32_HZ 0x07

#define GYRO_POWER_MODE_ADRESS 0x11
#define GYRO_POWER_MODE_NORMAL 0x00
#define GYRO_POWER_MODE_SUSPEND 0x80



#endif /* INC_BMP390_H_ */
