/*
 * m10-gps.c
 *
 *  Created on: Apr 8, 2025
 *      Author: benno
 */

#include "m10-gps.h"
#include "debug.h"
#include "stdbool.h"

static UART_HandleTypeDef *gps_uart;
static DMA_HandleTypeDef *gps_dma;
static uint32_t buffer_overflow_count = 0;
static uint8_t dma_buffer[GPS_BUFFER_SIZE] = {0};

static PARSE_STRUCT parse_struct = {0};
static GPS_NAV_PVT gps_nav_pvt = {0};

static bool UBX_ChecksumValid(uint8_t *ubx, uint16_t payload_len) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    // The checksum is calculated from index 2 to (6 + payload_len - 1)
    for (uint16_t i = 2; i < 6 + payload_len; i++) {
        ck_a = ck_a + ubx[i];
        ck_b = ck_b + ck_a;
    }

    uint8_t checksum_a = ubx[6 + payload_len];     // first checksum byte
    uint8_t checksum_b = ubx[6 + payload_len + 1]; // second checksum byte

    return (ck_a == checksum_a) && (ck_b == checksum_b);
}

static uint16_t GPS_GET_DMA_POSITION(){
	return __HAL_DMA_GET_COUNTER(gps_dma);
}

static void GPS_DECODE(){

	if(parse_struct.package_class == 0x01 && parse_struct.package_id == 0x07 && parse_struct.payload_len == 92 && UBX_ChecksumValid(parse_struct.ubx_package, parse_struct.payload_len)){
		memcpy(&gps_nav_pvt, &parse_struct.ubx_package[6], parse_struct.payload_len);
		STATUS_LED_GREEN_ON();
		//USB_PRINTLN("%d s", parse_struct.ubx_package[6 + 10]);
		USB_PRINTLN("%f m", gps_nav_pvt.hMSL / 1000.0);
	}
}

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	gps_uart = UARTx;
	gps_dma = UART_DMAx;
	HAL_UART_Receive_DMA(gps_uart, dma_buffer, GPS_BUFFER_SIZE);
}

void GPS_PARSE_BUFFER(void){
	STATUS_LED_GREEN_OFF();
	uint16_t dma_pos = GPS_GET_DMA_POSITION();
	if(buffer_overflow_count == parse_struct.parser_overflow_count){		// if the buffer has not wrapped around
		for(int i = parse_struct.parser_position; i < dma_pos - 6; i++){
			if(dma_buffer[i] == 0xB5 && dma_buffer[i + 1] == 0x62){

				parse_struct.package_class = dma_buffer[i + 2];				// extracting important data
				parse_struct.package_id = dma_buffer[i + 3];
				parse_struct.package_len = ((uint16_t)dma_buffer[i + 4] | ((uint16_t)dma_buffer[i + 5] << 8)) + 8;
				parse_struct.payload_len = ((uint16_t)dma_buffer[i + 4] | ((uint16_t)dma_buffer[i + 5] << 8));

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
