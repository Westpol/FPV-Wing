/*
 * crossfire.c
 *
 *  Created on: Apr 14, 2025
 *      Author: benno
 */

#include "crossfire.h"
#include "debug.h"
#include "stdbool.h"
#include "time-utils.h"
#include "main.h"

static UART_HandleTypeDef *crsf_uart;
static DMA_HandleTypeDef *crsf_dma;
static uint32_t buffer_wrap_around_count = 0;
__attribute__((aligned(32))) static uint8_t dma_buffer[CRSF_BUFFER_SIZE] = {0};
static CRSF_PARSE_STRUCT parser = {0};
CRSF_DATA crsf_data = {0};

static uint8_t crc8tab[256] = {
	    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

static void CRSF_DECODE_CHANNELS(const uint8_t *payload, uint16_t *channels) {		// converts 22 bytes to 16 11bit channels
    channels[0]  = (payload[0] | ((uint16_t)payload[1] << 8)) & 0x07FF;
    channels[1]  = ((payload[1] >> 3) | ((uint16_t)payload[2] << 5)) & 0x07FF;
    channels[2]  = ((payload[2] >> 6) | ((uint16_t)payload[3] << 2) | ((uint16_t)payload[4] << 10)) & 0x07FF;
    channels[3]  = ((payload[4] >> 1) | ((uint16_t)payload[5] << 7)) & 0x07FF;
    channels[4]  = ((payload[5] >> 4) | ((uint16_t)payload[6] << 4)) & 0x07FF;
    channels[5]  = ((payload[6] >> 7) | ((uint16_t)payload[7] << 1) | ((uint16_t)payload[8] << 9)) & 0x07FF;
    channels[6]  = ((payload[8] >> 2) | ((uint16_t)payload[9] << 6)) & 0x07FF;
    channels[7]  = ((payload[9] >> 5) | ((uint16_t)payload[10] << 3)) & 0x07FF;
    channels[8]  = (payload[11] | ((uint16_t)payload[12] << 8)) & 0x07FF;
    channels[9]  = ((payload[12] >> 3) | ((uint16_t)payload[13] << 5)) & 0x07FF;
    channels[10] = ((payload[13] >> 6) | ((uint16_t)payload[14] << 2) | ((uint16_t)payload[15] << 10)) & 0x07FF;
    channels[11] = ((payload[15] >> 1) | ((uint16_t)payload[16] << 7)) & 0x07FF;
    channels[12] = ((payload[16] >> 4) | ((uint16_t)payload[17] << 4)) & 0x07FF;
    channels[13] = ((payload[17] >> 7) | ((uint16_t)payload[18] << 1) | ((uint16_t)payload[19] << 9)) & 0x07FF;
    channels[14] = ((payload[19] >> 2) | ((uint16_t)payload[20] << 6)) & 0x07FF;
    channels[15] = ((payload[20] >> 5) | ((uint16_t)payload[21] << 3)) & 0x07FF;
}

static uint8_t crc8(const uint8_t * ptr, uint8_t len){		// outputs a CRC value from a given byte array
    uint8_t crc = 0;
    for(uint8_t i=0; i<len; i++){
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}

static uint16_t CRSF_GET_DMA_POSITION(){
    return CRSF_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(crsf_dma);
}

static bool VALIDATE_CRSF_CRC() {		// compares parser CRC to calculated CRC, outputs true or false upon match/mismatch
    // Payload length is already known
    uint8_t crc_input_len = parser.payload_len - 1;  // +1 for the Type byte
    uint8_t *crc_input_start = &parser.crsf_package[2];  // Type starts at byte 2
    uint8_t calculated_crc = crc8(crc_input_start, crc_input_len);

    return calculated_crc == parser.package_crc;
}

static uint16_t ATB(uint64_t abs_index_val){		// absolute index to buffer index
	return abs_index_val % CRSF_BUFFER_SIZE;
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

static void CRSF_DECODE(){
	if(parser.payload_type == 0x16 && VALIDATE_CRSF_CRC()){
		CRSF_DECODE_CHANNELS(&parser.crsf_package[3], crsf_data.channel);
		crsf_data.last_channel_update = MICROS();
	}
}

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	crsf_uart = UARTx;
	crsf_dma = UART_DMAx;
	if(HAL_UART_Receive_DMA(crsf_uart, dma_buffer, CRSF_BUFFER_SIZE) != HAL_OK){
		ERROR_HANDLER_BLINKS(1);
	}
}

void CRSF_HANDLE_TELEMETRY(){
	CRSF_SEND_TELEMETRY(0x0A);		// airspeed
}

void CRSF_SEND_TELEMETRY(uint8_t TELEMETRY_TYPE){
	uint8_t payload_data[3] = {TELEMETRY_TYPE, 0x05, 0x7C};
	uint8_t crc = crc8(payload_data, 3);
	uint8_t telemetry_data[6] = {0xEE, 0x04, TELEMETRY_TYPE, 0x05, 0x7C, crc};

	HAL_UART_Transmit_DMA(crsf_uart, telemetry_data, 6);
}

void CRSF_PARSE_BUFFER(){

	uintptr_t addr = (uintptr_t)dma_buffer;
	size_t size = CRSF_BUFFER_SIZE;

	if ((addr % CACHE_LINE_SIZE != 0) || (size % CACHE_LINE_SIZE != 0)) {
		ERROR_HANDLER_BLINKS(5);
	}

	SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

    uint64_t dma_pos_abs = CRSF_GET_DMA_POSITION() + (buffer_wrap_around_count * CRSF_BUFFER_SIZE);
    uint8_t stuck_counter = 0;

    while(stuck_counter < 50){
		if(dma_pos_abs - parser.parser_position > CRSF_BUFFER_SIZE){
			parser.parser_position = dma_pos_abs - (CRSF_BUFFER_SIZE / 2);
		}

		if(dma_pos_abs - 6 > parser.parser_position){
			if(dma_buffer[ATB(parser.parser_position)] == 0xC8){
				parser.package_header = dma_buffer[ATB(parser.parser_position)];
				parser.payload_len = dma_buffer[ATB(parser.parser_position + 1)];
				parser.payload_type = dma_buffer[ATB(parser.parser_position + 2)];
				parser.package_len = parser.payload_len + 2;

				if(parser.parser_position + parser.package_len > dma_pos_abs){
					return;		// returns if DMA hasn't read enough data yet, a.k.a. reached the end of the GPS message
				}

				parser.package_crc = dma_buffer[ATB(parser.parser_position + parser.package_len - 1)];

				MEMCPY_FROM_RINGBUFFER(parser.crsf_package, dma_buffer, ATB(parser.parser_position), parser.package_len, CRSF_BUFFER_SIZE);
				CRSF_DECODE();
				parser.parser_position += parser.package_len;
			}

			while(dma_pos_abs - 1 > parser.parser_position){
				if(dma_buffer[ATB(parser.parser_position)] == 0xC8){
					break;
				}

				parser.parser_position++;
			}
		}
		stuck_counter++;
    }
}

void CRSF_OVERFLOW_INCREMENT(){
	buffer_wrap_around_count++;
}
