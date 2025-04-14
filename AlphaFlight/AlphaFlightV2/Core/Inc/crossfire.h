/*
 * crossfire.h
 *
 *  Created on: Apr 14, 2025
 *      Author: benno
 */

#ifndef INC_CROSSFIRE_H_
#define INC_CROSSFIRE_H_

#include "stm32f7xx_hal.h"

#define CRSF_BUFFER_SIZE 128
#define MAX_PARSE_ITERATIONS 32  // Don't loop forever on garbage

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx);
void CRSF_PARSE_BUFFER();
void CRSF_OVERFLOW_INCREMENT();

#endif /* INC_CROSSFIRE_H_ */
