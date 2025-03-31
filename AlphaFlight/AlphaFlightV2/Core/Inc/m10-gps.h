/*
 * m10-gps.h
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */

#ifndef INC_M10_GPS_H_
#define INC_M10_GPS_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

#define BUFFER_SIZE 256

void GPS_INIT(UART_HandleTypeDef *HUARTx);
void GPS_DMA_READ_START(void);
bool NEW_GPS_DATA();
void CONVERT_DATA();

#endif /* INC_M10_GPS_H_ */
