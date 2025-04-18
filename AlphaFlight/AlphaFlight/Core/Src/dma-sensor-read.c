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

static uint8_t gyro_tx_buffer[7];
static uint8_t accel_tx_buffer[8];
static uint8_t baro_tx_buffer[8];

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
	if (hspi->State != HAL_SPI_STATE_READY) {
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  // Error LED
	    return;
	}
	gyro_tx_buffer[0] = GYRO_RATE_DATA_ADDRESS | READ_BYTE;
	accel_tx_buffer[0] = ACCEL_ACCELERATION_DATA_ADDRESS | READ_BYTE;
	baro_tx_buffer[0] = 0x04 | READ_BYTE;

	recieve_data_pointers.gyro = gyro_rx;
	recieve_data_pointers.accel = accel_rx;
	recieve_data_pointers.baro = baro_rx;
	GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
	GPIOC->BSRR = GPIO_PIN_4;
	sensor_point = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(hspi, gyro_tx_buffer, recieve_data_pointers.gyro, 7);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspix){
	if(hspix->Instance != hspi->Instance){
		return;
	}
	switch (sensor_point) {
		case 1:
			GYRO_NEW = true;
			sensor_point = 2;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive_DMA(hspi, accel_tx_buffer, recieve_data_pointers.accel, 8);
			break;
		case 2:
			sensor_point = 3;		// transmission ended until DMA_READ_SPI_SENSORS is called
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
			ACCEL_NEW = true;
			HAL_SPI_TransmitReceive_DMA(hspi, baro_tx_buffer, recieve_data_pointers.baro, 8);
			break;
		default:
			BARO_NEW = true;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
			break;
	}
}
