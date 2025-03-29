/*
 * dma-sensor-read.h
 *
 *  Created on: Mar 28, 2025
 *      Author: benno
 */

#ifndef INC_DMA_SENSOR_READ_H_
#define INC_DMA_SENSOR_READ_H_

#include "stm32f7xx_hal.h"
#include "stdbool.h"
#include "stdint.h"
#include "bmi088.h"
#include "bmp390.h"

typedef struct {
	uint8_t *gyro;
	uint8_t *accel;
	uint8_t *baro;
}Recieve_Data_Pointers;

bool GYRO_NEW_DATA();
bool ACCEL_NEW_DATA();
bool BARO_NEW_DATA();

void DMA_READ_SPI_SENSORS(SPI_HandleTypeDef *HSPIx, uint8_t *gyro_rx, uint8_t *accel_rx, uint8_t *baro_rx);

#endif /* INC_DMA_SENSOR_READ_H_ */
