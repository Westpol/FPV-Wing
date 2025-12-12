/*
 * m10-gps.c
 *
 *  Created on: Apr 8, 2025
 *      Author: benno
 */

#include "m10-gps.h"
#include "debug.h"
#include "stdbool.h"
#include "main.h"

static UART_HandleTypeDef *gps_uart;
static DMA_HandleTypeDef *gps_dma;
static uint32_t buffer_wrap_around_count = 0;
__attribute__((aligned(32))) static uint8_t dma_buffer[GPS_BUFFER_SIZE] = {0};

static GPS_PARSE_STRUCT parse_struct = {0};
GPS_DATA_T GPS_DATA = {0};
GPS_NAV_PVT gps_nav_pvt = {0};

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

static uint32_t GPS_DATETIME_TO_UNIX(uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t min, uint8_t sec) {
    if (year < 2025) return 0;

    // Days per month, non-leap year
    static const uint8_t days_in_month[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };

    uint32_t days = 0;

    // Add days for years since 1970
    for (uint16_t y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }

    // Add days for months in current year
    for (uint8_t m = 1; m < month; m++) {
        days += days_in_month[m - 1];
        if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
            days += 1; // Leap day
        }
    }

    // Add days in current month
    days += day - 1;

    return days * 86400 + hour * 3600 + min * 60 + sec;
}

static void GPS_CONVERT(){
	GPS_DATA.unix_timestamp = GPS_DATETIME_TO_UNIX(gps_nav_pvt.year, gps_nav_pvt.month, gps_nav_pvt.day, gps_nav_pvt.hour, gps_nav_pvt.min, gps_nav_pvt.sec);
	GPS_DATA.lat = (float)gps_nav_pvt.lat * 0.0000001;
	GPS_DATA.lon = (float)gps_nav_pvt.lon * 0.0000001;

	GPS_DATA.lat_int = gps_nav_pvt.lat;
	GPS_DATA.lon_int = gps_nav_pvt.lon;

	GPS_DATA.gspeed = (float)gps_nav_pvt.gSpeed / 1000.0;
	GPS_DATA.altitude = (float)gps_nav_pvt.hMSL / 1000.0;
	GPS_DATA.heading = (float)gps_nav_pvt.heading * 0.00001;
	GPS_DATA.velN = (float)gps_nav_pvt.velN / 1000.0;
	GPS_DATA.velE = (float)gps_nav_pvt.velE / 1000.0;
	GPS_DATA.velD = (float)gps_nav_pvt.velD / 1000.0;
	GPS_DATA.numSV = gps_nav_pvt.numSV;
	GPS_DATA.hAcc = gps_nav_pvt.hAcc;
	GPS_DATA.vAcc = gps_nav_pvt.vAcc;
	GPS_DATA.fix_type = gps_nav_pvt.fixType;
}

static void GPS_DECODE(){
	if(parse_struct.package_class == 0x01 && parse_struct.package_id == 0x07 && parse_struct.payload_len == 92 && UBX_ChecksumValid(parse_struct.ubx_package, parse_struct.payload_len)){
		memcpy(&gps_nav_pvt, &parse_struct.ubx_package[6], parse_struct.payload_len);
		GPS_CONVERT();
	}
}

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	gps_uart = UARTx;
	gps_dma = UART_DMAx;
	if(HAL_UART_Receive_DMA(gps_uart, dma_buffer, GPS_BUFFER_SIZE) != HAL_OK){
		ERROR_HANDLER_BLINKS(1);
	}
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
	uintptr_t addr = (uintptr_t)dma_buffer;
	size_t size = GPS_BUFFER_SIZE;

	if ((addr % CACHE_LINE_SIZE != 0) || (size % CACHE_LINE_SIZE != 0)) {
		#if DEBUG_ENABLED
		ERROR_HANDLER_BLINKS(5);
		#endif
		return;
	}

	SCB_InvalidateDCache_by_Addr((uint32_t*)addr, size);

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
