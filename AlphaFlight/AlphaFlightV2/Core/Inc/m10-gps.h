/*
 * m10-gps.h
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */

#ifndef INC_M10_GPS_H_
#define INC_M10_GPS_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct {
    float latitude;
    char lat_dir;
    float longitude;
    char lon_dir;
    int fix_quality;
    int satellites;
    float altitude;
    float speed;  // Knots
    char time[10];
    char date[10];
    uint8_t raw_buffer_data[256];
} GPS_Data;

#define BUFFER_SIZE 256
#define OVERFLOW_BUFFER_SIZE 128

void GPS_INIT(UART_HandleTypeDef *HUARTx);
void GPS_DMA_READ_START(void);
void EXTRACT_PACKAGES();
GPS_Data* GPS_GET_DATA(void);
void DUMP_BUFFER();

#endif /* INC_M10_GPS_H_ */
