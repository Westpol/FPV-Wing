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

static uint16_t GPS_GET_DMA_POSITION() {
    return GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_dma);
}

static void GPS_DECODE(){
	if(parse_struct.package_class == 0x01 && parse_struct.package_id == 0x07 && parse_struct.payload_len == 92){
		STATUS_LED_GREEN_ON();
		memcpy(&gps_nav_pvt, &parse_struct.ubx_package[6], parse_struct.payload_len);
		//USB_PRINTLN("%d s", parse_struct.ubx_package[6 + 10]);
		USB_PRINTLN("%d", gps_nav_pvt.fixType);
	}
}

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	gps_uart = UARTx;
	gps_dma = UART_DMAx;
	HAL_UART_Receive_DMA(gps_uart, dma_buffer, GPS_BUFFER_SIZE);
}

static void memcpy_from_ringbuffer(uint8_t *dest, const uint8_t *src_ring, uint16_t start, uint16_t len, uint16_t buf_size) {
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
	STATUS_LED_GREEN_OFF();
    uint16_t dma_pos = GPS_GET_DMA_POSITION();
    uint32_t dma_wrap = buffer_wrap_around_count;
    uint16_t pos = parse_struct.parser_position;
    int iterations = 0;

    while (iterations++ < MAX_PARSE_ITERATIONS) {
        uint16_t available;
        if (dma_wrap == parse_struct.parser_wrap_around_count) {
            available = (dma_pos >= pos) ? (dma_pos - pos) : 0;
        } else if (dma_wrap == parse_struct.parser_wrap_around_count + 1) {
            available = (GPS_BUFFER_SIZE - pos) + dma_pos;
        } else {
            parse_struct.parser_position = 0;
            parse_struct.parser_wrap_around_count = dma_wrap;
            return;
        }

        if (available < 8) return;

        uint8_t sync1 = dma_buffer[pos];
        uint8_t sync2 = dma_buffer[(pos + 1) % GPS_BUFFER_SIZE];

        if (sync1 == 0xB5 && sync2 == 0x62) {
            uint16_t len_low  = dma_buffer[(pos + 4) % GPS_BUFFER_SIZE];
            uint16_t len_high = dma_buffer[(pos + 5) % GPS_BUFFER_SIZE];
            uint16_t payload_len = len_low | (len_high << 8);
            uint16_t total_len = payload_len + 8;

            if (available < total_len) return;

            memcpy_from_ringbuffer(parse_struct.ubx_package, dma_buffer, pos, total_len, GPS_BUFFER_SIZE);

            // Optional: early sanity check on expected message class/ID/len before checksumming
            uint8_t msg_class = parse_struct.ubx_package[2];
            uint8_t msg_id = parse_struct.ubx_package[3];

            if (UBX_ChecksumValid(parse_struct.ubx_package, payload_len)) {
                parse_struct.package_class = msg_class;
                parse_struct.package_id = msg_id;
                parse_struct.payload_len = payload_len;

                GPS_DECODE();  // will turn LED on for valid NAV-PVT

                pos = (pos + total_len) % GPS_BUFFER_SIZE;
                parse_struct.parser_position = pos;
                parse_struct.parser_wrap_around_count = dma_wrap;
                continue;
            } else {
                pos = (pos + 1) % GPS_BUFFER_SIZE;
                parse_struct.parser_position = pos;
                continue;
            }
        } else {
            pos = (pos + 1) % GPS_BUFFER_SIZE;
            parse_struct.parser_position = pos;

            if (pos == dma_pos) return;
        }
    }
}


void GPS_OVERFLOW_INCREMENT(void){
	buffer_wrap_around_count++;
}
