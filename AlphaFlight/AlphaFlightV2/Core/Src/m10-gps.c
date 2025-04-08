/*
 * m10-gps.c
 *
 *  Created on: Apr 8, 2025
 *      Author: benno
 */

#include "m10-gps.h"
#include "debug.h"

static UART_HandleTypeDef *gps_uart;
static DMA_HandleTypeDef *gps_dma;
static uint32_t buffer_overflow_count = 0;
static uint8_t dma_buffer[GPS_BUFFER_SIZE] = {0};

static PARSE_STRUCT parse_struct = {0};
static GPS_NAV_PVT gps_nav_pvt = {0};

static uint16_t GPS_GET_DMA_POSITION(){
	return __HAL_DMA_GET_COUNTER(gps_dma);
}

static void GPS_DECODE(){
	/*
	for(int i = 0; i < parse_struct.package_len; i++){
		USB_PRINT("%X ", parse_struct.ubx_package[i]);
	}
	USB_PRINTLN("\r\n");*/
	USB_PRINTLN("%X, %X", parse_struct.package_class, parse_struct.package_id);
}

void GPS_DUMP(){
}

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	gps_uart = UARTx;
	gps_dma = UART_DMAx;
	HAL_UART_Receive_DMA(gps_uart, dma_buffer, GPS_BUFFER_SIZE);
}

void GPS_PARSE_BUFFER(void){
	uint16_t dma_pos = GPS_GET_DMA_POSITION();
	if(buffer_overflow_count == parse_struct.parser_overflow_count){		// if the buffer has not wrapped around
		for(int i = parse_struct.parser_position; i < dma_pos - 6; i++){
			if(dma_buffer[i] == 0xB5 && dma_buffer[i + 1] == 0x62){

				parse_struct.package_class = dma_buffer[i + 2];				// extracting important data
				parse_struct.package_id = dma_buffer[i + 3];
				parse_struct.package_len = ((uint16_t)dma_buffer[i + 4] | ((uint16_t)dma_buffer[i + 5] << 8)) + 8;

				if(dma_pos - i >= parse_struct.package_len){		// only proceed if the whole package is in the DMA buffer
					parse_struct.parser_position = i + 2;
					memcpy(&parse_struct.ubx_package[0], &dma_buffer[i], parse_struct.package_len);
					GPS_DECODE();
				}
				else{
					parse_struct.parser_position = i;
					return;		// wait until the buffer is big enough to decode the whole message
				}
			}
		}
	}
	else{
		parse_struct.parser_overflow_count = buffer_overflow_count;
		parse_struct.parser_position = 0;
	}

	// Look for 0xB5 0x62
	// Check if enough bytes follow for a full message
	// Validate length
	// Validate checksum
	// memcpy into ubx_package[]
	// decode if msg class = 0x01, id = 0x07 or 0x12
	// Move parser_position accordingly (handle wraparound)
}

void GPS_OVERFLOW_INCREMENT(void){
	buffer_overflow_count++;
}
