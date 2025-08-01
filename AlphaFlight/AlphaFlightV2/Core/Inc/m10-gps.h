/*
 * m10-gps.h
 *
 *  Created on: Apr 8, 2025
 *      Author: benno
 */

#ifndef INC_M10_GPS_H_
#define INC_M10_GPS_H_

#include "stdint.h"
#include "stm32f7xx_hal.h"

#define ROUND_UP_32(x) (((x) + 31) & ~31)
#define CACHE_LINE_SIZE 32

#define GPS_BUFFER_RAW_SIZE 512
#define GPS_BUFFER_SIZE ROUND_UP_32(GPS_BUFFER_RAW_SIZE)

#define MAX_PARSE_ITERATIONS 32  // Don't loop forever on garbage


typedef struct __attribute__((packed)){
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;        // 1e-7 deg
    int32_t  lat;        // 1e-7 deg
    int32_t  height;     // mm above ellipsoid
    int32_t  hMSL;       // mm above MSL
    uint32_t hAcc;       // mm
    uint32_t vAcc;       // mm
    int32_t  velN;       // mm/s
    int32_t  velE;       // mm/s
    int32_t  velD;       // mm/s
    int32_t  gSpeed;     // mm/s
    int32_t  heading;    // 1e-5 deg
    uint32_t sAcc;       // mm/s
    uint32_t headingAcc; // 1e-5 deg
    uint16_t pDOP;
    uint8_t  flags3;
    uint8_t  reserved1;
    uint32_t reserved2;
    int32_t  headVeh;    // 1e-5 deg
    int16_t  magDec;     // 1e-2 deg
    uint16_t magAcc;     // 1e-2 deg
} GPS_NAV_PVT;

typedef struct __attribute__((packed)){
	float lat;
	float lon;
	float gspeed;      // m/s
	float height;      // MSL, in meters
	float heading;     // deg
	float velN;        // m/s
	float velE;        // m/s
	float velD;        // m/s
	uint8_t numSV;
	uint8_t hAcc_dm;   // horizontal accuracy (decimeters)
	uint8_t vAcc_dm;   // vertical accuracy (decimeters)
} GPS_DATA;

typedef struct{
	uint64_t parser_position;
	uint8_t package_class;
	uint8_t package_id;
	uint16_t package_len;
	uint8_t payload_len;
	uint8_t ubx_package[GPS_BUFFER_SIZE];
}GPS_PARSE_STRUCT;

void GPS_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx);
void GPS_PARSE_BUFFER(void);
void GPS_OVERFLOW_INCREMENT(void);

#endif /* INC_M10_GPS_H_ */
