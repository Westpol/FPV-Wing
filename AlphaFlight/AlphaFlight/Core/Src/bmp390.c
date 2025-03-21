/*
 * bmp390.c
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 */

#include "bmp390.h"

int BMP_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN){
	HAL_GPIO_WritePin(GYRO_GPIOx, GYRO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GYRO_GPIOx, GYRO_PIN, GPIO_PIN_SET);
}
