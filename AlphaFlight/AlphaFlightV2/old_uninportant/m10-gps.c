/*
 * m10-gps.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "m10-gps.h"
#include "debug.h"
#include "string.h"
#include "stdlib.h"

static uint8_t dma_buffer[BUFFER_SIZE] = {0}; // DMA Buffer
static uint8_t circular_buffer[OVERFLOW_BUFFER_SIZE] = {0}; // Circular Buffer
static uint16_t buffer_index = 0; // Index for Circular Buffer

static UART_HandleTypeDef *huart;

static GPS_Data gps_data = {0};

/**
 * @brief Initialize GPS UART interface.
 */
void GPS_INIT(UART_HandleTypeDef *HUARTx){
	huart = HUARTx;
}

/**
 * @brief Start receiving GPS data via DMA.
 */
void GPS_DMA_READ_START(void){
	HAL_UART_Receive_DMA(huart, dma_buffer, BUFFER_SIZE);
}

/**
 * @brief Returns pointer to the latest GPS data struct.
 */
GPS_Data* GPS_GET_DATA(void) {
    return &gps_data;
}

/**
 * @brief Parses a complete NMEA sentence stored in circular_buffer.
 */
static void PARSE_PACKAGE(){
	circular_buffer[buffer_index] = '\0'; // Null-terminate the string

	if (strncmp((char*)circular_buffer, "$GNGGA", 6) == 0) {
    	STATUS_LED_GREEN_ON();
        char *token = strtok((char*)circular_buffer, ",");
        int field = 0;

        while (token != NULL) {
            switch (field) {
                case 1: strncpy(gps_data.time, token, 9); break;
                case 2: gps_data.latitude = atof(token) / 100.0; break;
                case 3: gps_data.lat_dir = token[0]; break;
                case 4: gps_data.longitude = atof(token) / 100.0; break;
                case 5: gps_data.lon_dir = token[0]; break;
                case 6: gps_data.fix_quality = atoi(token); break;
                case 7: gps_data.satellites = atoi(token); break;
                case 9: gps_data.altitude = atof(token); break;
            }
            token = strtok(NULL, ",");
            field++;
        }
    }
    else if (strncmp((char*)circular_buffer, "$GNRMC", 6) == 0) {
        char *token = strtok((char*)circular_buffer, ",");
        int field = 0;

        while (token != NULL) {
            switch (field) {
                case 1: strncpy(gps_data.time, token, 9); break;
                case 2: if (token[0] != 'A') return; break; // Check if fix is valid
                case 3: gps_data.latitude = atof(token) / 100.0; break;
                case 4: gps_data.lat_dir = token[0]; break;
                case 5: gps_data.longitude = atof(token) / 100.0; break;
                case 6: gps_data.lon_dir = token[0]; break;
                case 7: gps_data.speed = atof(token); break;
                case 9: strncpy(gps_data.date, token, 9); break;
            }
            token = strtok(NULL, ",");
            field++;
        }
    }

    buffer_index = 0; // Reset buffer index for next message
}

/**
 * @brief Processes new data from DMA and extracts complete NMEA sentences.
 */
static void EXTRACT_PACKAGES(uint8_t *new_data, uint16_t length){
	for (uint16_t i = 0; i < length; i++) {
		if (new_data[i] == '$') {
			// If a new sentence starts, process the previous one (if any)
			if (buffer_index > 0) {
				PARSE_PACKAGE();
			}
			buffer_index = 0; // Reset buffer index for new sentence
		}

		// Store character in circular buffer
		circular_buffer[buffer_index] = new_data[i];
		buffer_index++;

		// If sentence is complete, parse it
		if (new_data[i] == '\n') {
			PARSE_PACKAGE();
		}

		// Prevent overflow
		if (buffer_index >= OVERFLOW_BUFFER_SIZE) {
			buffer_index = 0;
		}
	}
}

/**
 * @brief DMA Full Buffer Complete Callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *HUARTx) {
    if (HUARTx->Instance == huart->Instance) {
    	EXTRACT_PACKAGES(&dma_buffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
    }
}

/**
 * @brief DMA Half Buffer Complete Callback
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *HUARTx){
	if (HUARTx->Instance == huart->Instance) {
		EXTRACT_PACKAGES(dma_buffer, BUFFER_SIZE / 2);
	}
}

/**
 * @brief Dumps raw buffer data for debugging.
 */
void DUMP_BUFFER(){
	memcpy(gps_data.raw_buffer_data, dma_buffer, 256);
}
