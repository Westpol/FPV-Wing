/*
 * dma-sensor-read.h
 *
 *  Created on: Mar 28, 2025
 *      Author: benno
 */

#ifndef INC_DMA_SENSOR_READ_H_
#define INC_DMA_SENSOR_READ_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "bmi088.h"
#include "bmp390.h"

void DMA_READ_SPI_SENSORS(SPI_HandleTypeDef *HSPIx);

#endif /* INC_DMA_SENSOR_READ_H_ */
