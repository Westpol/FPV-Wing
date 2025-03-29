/*
 * sma-sensor-read.c
 *
 *  Created on: Mar 28, 2025
 *      Author: benno
 */

#include "dma-sensor-read.h"

volatile uint8_t sensor_point = 0;
SPI_HandleTypeDef *hspi;
static Recieve_Data_Pointers recieve_data_pointers;

static bool GYRO_NEW = false;
static bool ACCEL_NEW = false;
static bool BARO_NEW = false;

bool GYRO_NEW_DATA(){
	if(GYRO_NEW == true){
		GYRO_NEW = false;
		return true;
	}
	else{
		return false;
	}
}
bool ACCEL_NEW_DATA(){
	if(ACCEL_NEW == true){
		ACCEL_NEW = false;
		return true;
	}
	else{
		return false;
	}
}
bool BARO_NEW_DATA(){
	if(BARO_NEW == true){
		BARO_NEW = false;
		return true;
	}
	else{
		return false;
	}
}

void DMA_READ_SPI_SENSORS(SPI_HandleTypeDef *HSPIx, uint8_t *gyro_rx, uint8_t *accel_rx, uint8_t *baro_rx){
	hspi = HSPIx;
	recieve_data_pointers.gyro = gyro_rx;
	recieve_data_pointers.accel = accel_rx;
	recieve_data_pointers.baro = baro_rx;
	GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
	GPIOC->BSRR = GPIO_PIN_4;
	uint8_t tx_buffer[7] = {GYRO_RATE_DATA_ADDRESS, 0, 0, 0, 0, 0, 0};
	sensor_point = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer, recieve_data_pointers.gyro, 7);
}

void HAL_SPI_RxTxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance != SPI1) return;
	switch (sensor_point) {
		case 1:
			sensor_point = 2;
			uint8_t tx_buffer_accel[8] = {ACCEL_ACCELERATION_DATA_ADDRESS, 0, 0, 0, 0, 0, 0, 0};
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer_accel, recieve_data_pointers.accel, 8);
			break;
		case 2:
			sensor_point = 3;		// transmission ended until DMA_READ_SPI_SENSORS is called
			uint8_t tx_buffer_baro[8] = {0x04 | READ_BYTE, 0, 0, 0, 0, 0, 0, 0};
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive_DMA(hspi, tx_buffer_baro, recieve_data_pointers.baro, 8);
			break;
		default:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			break;
	}
}
