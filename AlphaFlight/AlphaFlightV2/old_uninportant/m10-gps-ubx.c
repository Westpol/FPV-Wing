/*
 * m10-gps-ubx.c
 *
 *  Created on: Apr 3, 2025
 *      Author: benno
 */

#include "m10-gps-ubx.h"

static CircularBuffer gpsBuffer = { .head = 0, .tail = 0 };
static UART_HandleTypeDef *gpsUart;
static uint8_t dmaBuffer[GPS_BUFFER_SIZE];
GPS_Data_t gpsData;

void GPS_Init(UART_HandleTypeDef *huart) {
    gpsUart = huart;
    HAL_UART_Receive_DMA(gpsUart, dmaBuffer, GPS_BUFFER_SIZE);
}

void GPS_UART_DMA_HalfComplete_Callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == gpsUart->Instance) {
        for (uint16_t i = 0; i < GPS_BUFFER_SIZE / 2; i++) {
            uint16_t next = (gpsBuffer.head + 1) % GPS_BUFFER_SIZE;
            if (next != gpsBuffer.tail) {
                gpsBuffer.buffer[gpsBuffer.head] = dmaBuffer[i];
                gpsBuffer.head = next;
            }
        }
    }
}

void GPS_UART_DMA_Complete_Callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == gpsUart->Instance) {
        for (uint16_t i = GPS_BUFFER_SIZE / 2; i < GPS_BUFFER_SIZE; i++) {
            uint16_t next = (gpsBuffer.head + 1) % GPS_BUFFER_SIZE;
            if (next != gpsBuffer.tail) {
                gpsBuffer.buffer[gpsBuffer.head] = dmaBuffer[i];
                gpsBuffer.head = next;
            }
        }
        HAL_UART_Receive_DMA(gpsUart, dmaBuffer, GPS_BUFFER_SIZE);
    }
}

bool GPS_Available(void) {
    return gpsBuffer.head != gpsBuffer.tail;
}

uint8_t GPS_ReadByte(void) {
    if (!GPS_Available()) return 0;
    uint8_t data = gpsBuffer.buffer[gpsBuffer.tail];
    gpsBuffer.tail = (gpsBuffer.tail + 1) % GPS_BUFFER_SIZE;
    return data;
}

void GPS_Process(void) {
    static enum { FIND_SYNC1, FIND_SYNC2, READ_HEADER, READ_PAYLOAD, READ_CHECKSUM } state = FIND_SYNC1;
    static uint8_t ubxMsg[100]; // Temporary storage for UBX messages
    static uint16_t index = 0, payloadLength = 0;

    while (GPS_Available()) {
        uint8_t byte = GPS_ReadByte();
        switch (state) {
            case FIND_SYNC1:
                if (byte == UBX_SYNC_1) state = FIND_SYNC2;
                break;
            case FIND_SYNC2:
                if (byte == UBX_SYNC_2) {
                    index = 0;
                    state = READ_HEADER;
                } else {
                    state = FIND_SYNC1;
                }
                break;
            case READ_HEADER:
                ubxMsg[index++] = byte;
                if (index == 4) {
                    payloadLength = ubxMsg[2] | (ubxMsg[3] << 8);
                    state = READ_PAYLOAD;
                }
                break;
            case READ_PAYLOAD:
                ubxMsg[index++] = byte;
                if (index == 4 + payloadLength) state = READ_CHECKSUM;
                break;
            case READ_CHECKSUM:
                ubxMsg[index++] = byte;
                if (index == 6 + payloadLength) {
                    // Process UBX message here and update gpsData
                    state = FIND_SYNC1;
                }
                break;
        }
    }
}
