/*
 * crossfire.c
 *
 *  Created on: Apr 14, 2025
 *      Author: benno
 */

#include "crossfire.h"
#include "debug.h"

static UART_HandleTypeDef *crsf_uart;
static DMA_HandleTypeDef *crsf_dma;
static uint32_t buffer_wrap_around_count = 0;
static uint8_t dma_buffer[CRSF_BUFFER_SIZE] = {0};

static uint16_t GPS_GET_DMA_POSITION() {
    return CRSF_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(crsf_dma);
}

void CRSF_INIT(UART_HandleTypeDef *UARTx, DMA_HandleTypeDef *UART_DMAx){
	crsf_uart = UARTx;
	crsf_dma = UART_DMAx;
	HAL_UART_Receive_DMA(crsf_uart, dma_buffer, CRSF_BUFFER_SIZE);
}

void CRSF_PARSE_BUFFER(){
	USB_PRINT_HEX(dma_buffer, CRSF_BUFFER_SIZE);
}

void CRSF_OVERFLOW_INCREMENT(){
	buffer_wrap_around_count++;
}
