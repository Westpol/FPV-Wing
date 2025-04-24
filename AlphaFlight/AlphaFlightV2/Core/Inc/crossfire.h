/*
 * crossfire.h
 *
 *  Created on: Apr 14, 2025
 *      Author: benno
 */

#ifndef INC_CROSSFIRE_H_
#define INC_CROSSFIRE_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"

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

typedef struct{
	uint16_t channel[16];
	uint64_t last_channel_update;
	uint16_t rssi;
}CRSF_DATA;

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx);
void CRSF_HANDLE_TELEMETRY();
void CRSF_SEND_TELEMETRY(uint8_t TELEMETRY_TYPE);
void CRSF_PARSE_BUFFER();
void CRSF_OVERFLOW_INCREMENT();
CRSF_DATA* CRSF_GET_DATA_STRUCT();

#endif /* INC_CROSSFIRE_H_ */
