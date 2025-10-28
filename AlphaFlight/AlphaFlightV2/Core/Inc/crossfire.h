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

#define ROUND_UP_32(x) (((x) + 31) & ~31)
#define CACHE_LINE_SIZE 32

#define CRSF_BUFFER_RAW_SIZE 128
#define CRSF_BUFFER_SIZE ROUND_UP_32(CRSF_BUFFER_RAW_SIZE)
#define MAX_PARSE_ITERATIONS 32  // Don't loop forever on garbage

typedef struct{
	uint8_t throttle;
	uint8_t pitch;
	uint8_t roll;
	uint8_t arm_switch;
	uint8_t mode_switch;
}CRSF_CHANNEL;

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
	uint16_t channel_raw[16];
	float channel_norm[16];
	uint64_t last_channel_update;
	uint16_t rssi;
}CRSF_DATA;

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx);
void CRSF_HANDLE_TELEMETRY();
void CRSF_SEND_TELEMETRY(uint8_t TELEMETRY_TYPE);
void CRSF_PARSE_BUFFER();
void CRSF_OVERFLOW_INCREMENT();

#endif /* INC_CROSSFIRE_H_ */
