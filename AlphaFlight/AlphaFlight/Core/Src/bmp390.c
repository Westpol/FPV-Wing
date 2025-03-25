/*
 * bmp390.c
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 */

#include "bmp390.h"

static void read_address(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t *txbuffer, uint8_t *rxbuffer, uint8_t readLength){
	txbuffer[0] = txbuffer[0] | READ_BYTE;
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txbuffer, rxbuffer, readLength, 100);
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_SET);
}

static void write_address(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t reg, uint8_t data){
	uint8_t txbuffer[2] = {reg & WRITE_BYTE, data};
	uint8_t rxbuffer[2];
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txbuffer, rxbuffer, 2, 100);
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_SET);
}


int BMP_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN){
	HAL_GPIO_WritePin(GYRO_GPIOx, GYRO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GYRO_GPIOx, GYRO_PIN, GPIO_PIN_SET);
	return 0;
}
