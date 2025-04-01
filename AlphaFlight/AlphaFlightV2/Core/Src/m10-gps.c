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

static uint8_t dma_buffer[BUFFER_SIZE] = {0};
static uint8_t first_half_buffer[BUFFER_SIZE / 2] = {0};
static uint8_t second_half_buffer[BUFFER_SIZE / 2] = {0};
static bool first_half_new_data = false;
static bool second_half_new_data = false;
static uint8_t buffer[OVERFLOW_BUFFER_SIZE] = {0};
static uint8_t buffer_index = 0;

static UART_HandleTypeDef *huart;

static GPS_Data gps_data = {0};

static void MOVE_DATA_FIRST_HALF(){
	memcpy(first_half_buffer, dma_buffer, BUFFER_SIZE / 2);
}

static void MOVE_DATA_SECOND_HALF(){
	memcpy(second_half_buffer, &dma_buffer[BUFFER_SIZE / 2], BUFFER_SIZE / 2);
}

void GPS_INIT(UART_HandleTypeDef *HUARTx){
	huart = HUARTx;
}

void GPS_DMA_READ_START(void){
	HAL_UART_Receive_DMA(huart, dma_buffer, BUFFER_SIZE);
}

GPS_Data* GPS_GET_DATA(void) {
    return &gps_data;
}

static void PARSE_PACKAGE(){
	buffer[buffer_index] = '\0';

    if (strncmp((char*)buffer, "$GNGGA", 6) == 0) {
    	STATUS_LED_GREEN_ON();
        char *token = strtok((char*)buffer, ",");
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
    else if (strncmp((char*)buffer, "$GNRMC", 6) == 0) {
        char *token = strtok((char*)buffer, ",");
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

    buffer_index = 0; // Reset buffer index after parsing
}

void EXTRACT_PACKAGES(){
	if(first_half_new_data){
		for(int i = 0; i < BUFFER_SIZE / 2; i++){
			if(first_half_buffer[i] == '$'){
				PARSE_PACKAGE();
			}
			buffer[buffer_index] = first_half_buffer[i];
			buffer_index += 1;
			if(buffer_index >= OVERFLOW_BUFFER_SIZE) buffer_index = 0;
		}
		first_half_new_data = false;
	}
	if(second_half_new_data){
		for(int i = 0; i < BUFFER_SIZE / 2; i++){
			if(second_half_buffer[i] == '$'){
				PARSE_PACKAGE();
			}
			buffer[buffer_index] = second_half_buffer[i];
			buffer_index += 1;
			if(buffer_index >= OVERFLOW_BUFFER_SIZE) buffer_index = 0;
		}
		second_half_new_data = false;
	}
}

void DUMP_BUFFER(){
	memcpy(gps_data.raw_buffer_data, dma_buffer, 256);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *HUARTx) {
    if(HUARTx->Instance == huart->Instance){
    	MOVE_DATA_SECOND_HALF();
    	second_half_new_data = true;
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *HUARTx){
	if(HUARTx->Instance == huart->Instance){
		MOVE_DATA_FIRST_HALF();
		first_half_new_data = true;
	}
}
