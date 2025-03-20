/*
 * bmi088.c
 *
 *  Created on: Mar 19, 2025
 *      Author: benno
 */

#include "bmi088.h"

static SPI_HandleTypeDef *bmi088_spi;
static GPIO_TypeDef *gyro_port;
static uint16_t gyro_pin;
static GPIO_TypeDef *accel_port;
static uint16_t accel_pin;

Gyro_Data gyro_data;

int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN){
	gyro_data.gyro_x_raw = 0;
	gyro_data.gyro_y_raw = 0;
	gyro_data.gyro_z_raw = 0;
	bmi088_spi = hspi;
	gyro_port = GYRO_GPIOx;
	gyro_pin = GYRO_PIN;
	accel_port = ACCEL_GPIOx;
	accel_pin = ACCEL_PIN;
	uint8_t tx_buffer[2] = {0x00 | READ_BYTE, 0x00};
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

	// dummy setup here
	tx_buffer[0] = GYRO_RANGE_ADRESS & WRITE_BYTE;		// setting up range
	tx_buffer[1] = GYRO_RANGE_2000_DEG_PER_SECOND;
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);

	tx_buffer[0] = GYRO_ODR_FILTER_ADRESS & WRITE_BYTE;		// setting up filter
	tx_buffer[1] = GYRO_ODR_100_HZ_FILTER_12_HZ;
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);

	tx_buffer[0] = GYRO_POWER_MODE_ADRESS & WRITE_BYTE;		// setting up power mode
	tx_buffer[1] = GYRO_POWER_MODE_NORMAL;
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);
	// need to add real Setup here

	return 0;

}
void BMI_READ_GYRO_DATA(){
	uint8_t tx_buffer[7] = {0x02 | READ_BYTE, 0, 0, 0, 0, 0, 0};
	uint8_t rx_buffer[7];

	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 7, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);

	gyro_data.gyro_x_raw = ((int16_t)rx_buffer[1] << 8) | rx_buffer[2];
	gyro_data.gyro_y_raw = ((int16_t)rx_buffer[3] << 8) | rx_buffer[4];
	gyro_data.gyro_z_raw = ((int16_t)rx_buffer[5] << 8) | rx_buffer[6];

}

double BMI_GET_GYRO_X(){
	return (gyro_data.gyro_x_raw / 32767.0) * 2000.0;
}
double BMI_GET_GYRO_Y(){
	return (gyro_data.gyro_y_raw / 32767.0) * 2000.0;
}
double BMI_GET_GYRO_Z(){
	return (gyro_data.gyro_z_raw / 32767.0) * 2000.0;
}

int16_t BMI_GET_GYRO_X_RAW(){
	return gyro_data.gyro_x_raw;
}
int16_t BMI_GET_GYRO_Y_RAW(){
	return gyro_data.gyro_y_raw;
}
int16_t BMI_GET_GYRO_Z_RAW(){
	return gyro_data.gyro_z_raw;
}

void BMI_GYRO_SOFT_RESET(){
	uint8_t tx_buffer[2] = {0x11 & WRITE_BYTE, 0xB6};
	uint8_t rx_buffer[2];
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bmi088_spi, tx_buffer, rx_buffer, 2, 100);
	HAL_GPIO_WritePin(gyro_port, gyro_pin, GPIO_PIN_SET);		// IMPORTANT: WAIT FOR AT LEAST 30ms BEFORE COMMUNICATING AGAIN!
}
