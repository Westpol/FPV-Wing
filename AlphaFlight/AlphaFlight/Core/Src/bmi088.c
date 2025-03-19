/*
 * bmi088.c
 *
 *  Created on: Mar 19, 2025
 *      Author: benno
 */

#include "bmi088.h"

static SPI_HandleTypeDef *bmi088_spi;
static GPIO_TypeDef *gyro_port;
static GPIO_TypeDef *gyro_pin;
static GPIO_TypeDef *accel_port;
static GPIO_TypeDef *accel_pin;

int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint8_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint8_t ACCEL_PIN){
	bmi088_spi = hspi;
	gyro_port = GYRO_GPIOx;
	gyro_pin = GYRO_PIN;
	accel_port = ACCEL_GPIOx;
	accel_pin = ACCEL_PIN;
	uint8_t tx_buffer[2] = {0x00 | 0x80, 0x00};
	uint8_t rx_buffer[2] = {0x00, 0x00};

	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);


	if(rx_buffer[1] != 0x0F){
		return 1;
	}

	HAL_GPIO_WritePin(accel_port, accel_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(accel_port, accel_pin, GPIO_PIN_SET);

	if(rx_buffer[1] != 0x1E){
		return 1;
	}
	//need to add Setup here

	return 0;

}
void BMI_READ_GYRO_DATA(){

}

void BMI_GET_GYRO_X(){

}
void BMI_GET_GYRO_Y(){

}
void BMI_GET_GYRO_Z(){

}

void BMI_GET_X_RAW(){

}
void BMI_GET_Y_RAW(){

}
void BMI_GET_Z_RAW(){

}
