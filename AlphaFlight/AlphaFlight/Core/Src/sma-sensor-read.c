/*
 * sma-sensor-read.c
 *
 *  Created on: Mar 28, 2025
 *      Author: benno
 */

#include "dma-sensor-read.h"

volatile uint8_t state = 0;
SPI_HandleTypeDef *hspi;

void DMA_READ_SPI_SENSORS(SPI_HandleTypeDef *HSPIx){
	hspi = HSPIx;
	GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
	GPIOC->BSRR = GPIO_PIN_4;
	HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, Size);
}

void HAL_SPI_RxTxCpltCallback(SPI_HandleTypeDef *hspi){

}
