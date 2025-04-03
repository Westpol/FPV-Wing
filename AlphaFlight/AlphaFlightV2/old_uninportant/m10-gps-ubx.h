/*
 * m10-gps-ubx.h
 *
 *  Created on: Apr 3, 2025
 *      Author: benno
 */

#ifndef INC_M10_GPS_UBX_H_
#define INC_M10_GPS_UBX_H_


#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_BUFFER_SIZE 512  // Circular buffer size
#define UBX_SYNC_1 0xB5
#define UBX_SYNC_2 0x62

// Circular buffer structure
typedef struct {
    uint8_t buffer[GPS_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} CircularBuffer;

// GPS data structure
typedef struct {
    uint32_t iTOW;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint8_t numSV;
} GPS_Data_t;

void GPS_Init(UART_HandleTypeDef *huart);
void GPS_Process(void);
void GPS_UART_IRQHandler(UART_HandleTypeDef *huart);
void GPS_UART_DMA_HalfComplete_Callback(UART_HandleTypeDef *huart);
void GPS_UART_DMA_Complete_Callback(UART_HandleTypeDef *huart);
bool GPS_Available(void);
uint8_t GPS_ReadByte(void);
extern GPS_Data_t gpsData;


#endif /* INC_M10_GPS_UBX_H_ */
