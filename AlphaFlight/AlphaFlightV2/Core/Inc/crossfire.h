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

typedef struct{
	uint64_t parser_position;
	uint8_t package_header;
	uint8_t payload_len;
	uint8_t payload_type;
	uint8_t package_len;
	uint8_t package_crc;
	uint8_t crsf_package[CRSF_BUFFER_SIZE];
}CRSF_PARSE_STRUCT;

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx);
void CRSF_PARSE_BUFFER();
void CRSF_OVERFLOW_INCREMENT();

#endif /* INC_CROSSFIRE_H_ */
