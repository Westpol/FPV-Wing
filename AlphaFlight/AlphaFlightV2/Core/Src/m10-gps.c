/*
 * m10-gps.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "m10-gps.h"
#include "debug.h"

static uint8_t buffer[BUFFER_SIZE];
static UART_HandleTypeDef *huart;

void GPS_INIT(UART_HandleTypeDef *HUARTx){
	huart = HUARTx;
}

void GPS_DMA_READ_START(void){
	HAL_UART_Receive_DMA(huart, buffer, BUFFER_SIZE);
}

bool NEW_GPS_DATA(){
	return true;
}

void CONVERT_DATA(){
	for(int i = 0; i < BUFFER_SIZE; i++){
		USB_PRINTLN("%d", buffer[i]);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *HUARTx) {
    if (HUARTx->Instance == huart->Instance) {
    	CONVERT_DATA();
    }
}
