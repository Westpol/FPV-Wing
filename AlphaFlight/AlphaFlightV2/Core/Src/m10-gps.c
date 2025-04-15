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
static uint32_t buffer_wrap_around_count = 0;
static uint8_t dma_buffer[GPS_BUFFER_SIZE] = {0};

static GPS_PARSE_STRUCT parse_struct = {0};
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

static uint16_t GPS_GET_DMA_POSITION() {
    return GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_dma);
}

static void GPS_DECODE(){
	if(parse_struct.package_class == 0x01 && parse_struct.package_id == 0x07 && parse_struct.payload_len == 92 && UBX_ChecksumValid(parse_struct.ubx_package, parse_struct.payload_len)){
		memcpy(&gps_nav_pvt, &parse_struct.ubx_package[6], parse_struct.payload_len);
	}
}

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	gps_uart = UARTx;
	gps_dma = UART_DMAx;
	HAL_UART_Receive_DMA(gps_uart, dma_buffer, GPS_BUFFER_SIZE);
}

static uint16_t ATB(uint64_t abs_index_val){		// absolute to buffer index values
	return abs_index_val % GPS_BUFFER_SIZE;
}

static void MEMCPY_FROM_RINGBUFFER(uint8_t *dest, const uint8_t *src_ring, uint16_t start, uint16_t len, uint16_t buf_size) {
    if (start + len <= buf_size) {
        memcpy(dest, &src_ring[start], len);
    } else {
        uint16_t first_part = buf_size - start;
        uint16_t second_part = len - first_part;
        memcpy(dest, &src_ring[start], first_part);
        memcpy(dest + first_part, &src_ring[0], second_part);
    }
}

void GPS_PARSE_BUFFER(void) {
    uint64_t dma_pos_abs = GPS_GET_DMA_POSITION() + (buffer_wrap_around_count * GPS_BUFFER_SIZE);
    uint8_t stuck_counter = 0;

    while(stuck_counter < 50){
		if(dma_pos_abs - parse_struct.parser_position > GPS_BUFFER_SIZE){
			parse_struct.parser_position = dma_pos_abs - (GPS_BUFFER_SIZE / 2);
		}

		if(dma_pos_abs - 6 > parse_struct.parser_position){
			if(dma_buffer[ATB(parse_struct.parser_position)] == 0xB5 && dma_buffer[ATB(parse_struct.parser_position + 1)] == 0x62){
				parse_struct.package_class = dma_buffer[ATB(parse_struct.parser_position + 2)];
				parse_struct.package_id = dma_buffer[ATB(parse_struct.parser_position + 3)];
				parse_struct.package_len = ((uint16_t)dma_buffer[ATB(parse_struct.parser_position + 4)] | ((uint16_t)dma_buffer[ATB(parse_struct.parser_position + 5)] << 8)) + 8;
				parse_struct.payload_len = ((uint16_t)dma_buffer[ATB(parse_struct.parser_position + 4)] | ((uint16_t)dma_buffer[ATB(parse_struct.parser_position + 5)] << 8));

				if(parse_struct.parser_position + parse_struct.package_len > dma_pos_abs){
					return;		// returns if DMA hasn't read enough data yet, a.k.a. reached the end of the GPS message
				}

				MEMCPY_FROM_RINGBUFFER(parse_struct.ubx_package, dma_buffer, ATB(parse_struct.parser_position), parse_struct.package_len, GPS_BUFFER_SIZE);
				GPS_DECODE();
				parse_struct.parser_position += parse_struct.package_len;
			}

			while(dma_pos_abs - 1 > parse_struct.parser_position){
				if(dma_buffer[ATB(parse_struct.parser_position)] == 0xB5 && dma_buffer[ATB(parse_struct.parser_position + 1)] == 0x62){
					break;
				}

				parse_struct.parser_position++;
			}
		}
		stuck_counter++;
    }
}


void GPS_OVERFLOW_INCREMENT(void){
	buffer_wrap_around_count++;
}


GPS_NAV_PVT* GPS_NAV_PVT_STRUCT_POINTER(){
	return &gps_nav_pvt;
}
