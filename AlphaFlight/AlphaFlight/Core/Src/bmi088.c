/*
 * bmi088.c
 *
 *  Created on: Mar 19, 2025
 *      Author: benno
 *
 *      TODO: THE FUCKING ACCELEROMETER HAS TO BE READ FROM THE THIRD BYTE IN READ MODE WHAT THE FUCK
 */

#include "bmi088.h"

static SPI_HandleTypeDef *bmi088_spi;
static GPIO_TypeDef *gyro_port;
static uint16_t gyro_pin;
static GPIO_TypeDef *accel_port;
static uint16_t accel_pin;
bool gyro_accel_calibration = false;
bool accel_right_for_calibration = false;
bool fucked_accel_reading = false;

uint32_t last_microseconds = 0;		// time where x was calculated, time where y was calculated, time where z was calculated

Gyro_Data gyro_data = {0};
Accel_Data accel_data = {0};


void read_address(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t *txbuffer, uint8_t *rxbuffer, uint8_t readLength){
	txbuffer[0] = txbuffer[0] | READ_BYTE;
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txbuffer, rxbuffer, readLength, 100);
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_SET);
}

void write_address(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t reg, uint8_t data){
	uint8_t txbuffer[2] = {reg & WRITE_BYTE, data};
	uint8_t rxbuffer[2];
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspi, txbuffer, rxbuffer, 2, 100);
	HAL_GPIO_WritePin(DEVICE_GPIOx, DEVICE_PIN, GPIO_PIN_SET);
}


int BMI_INIT_GYRO(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN){
	bmi088_spi = hspi;
	gyro_port = GYRO_GPIOx;
	gyro_pin = GYRO_PIN;

	uint8_t tx_buffer[2] = {0x00, 0x00};
	uint8_t rx_buffer[2] = {0x00, 0x00};

	read_address(bmi088_spi, gyro_port, gyro_pin, tx_buffer, rx_buffer, 2);		// reads chip ID, if chip doesn't correctly return ID, return error code

	if(rx_buffer[1] != 0x0F){
		return 1;
	}

	// dummy setup here
	write_address(bmi088_spi, gyro_port, gyro_pin, GYRO_RANGE_ADDRESS, GYRO_RANGE_2000_DEG_PER_SECOND);		// setting gyro range
	write_address(bmi088_spi, gyro_port, gyro_pin, GYRO_ODR_FILTER_ADDRESS, GYRO_ODR_100_HZ_FILTER_12_HZ);	// setting up filter
	write_address(bmi088_spi, gyro_port, gyro_pin, GYRO_POWER_MODE_ADDRESS, GYRO_POWER_MODE_NORMAL);		// setting up power mode
	write_address(bmi088_spi, gyro_port, gyro_pin, 0x15, 0x80);		// enabling interrupt
	write_address(bmi088_spi, gyro_port, gyro_pin, 0x16, 0b00001011);		// INT4 IO Config
	write_address(bmi088_spi, gyro_port, gyro_pin, 0x18, 0x80);		// data ready interrupt mapped to INT4
	// need to add real Setup here

	return 0;
}

int BMI_INIT_ACCEL(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN){
	bmi088_spi = hspi;
	accel_port = ACCEL_GPIOx;
	accel_pin = ACCEL_PIN;

	uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};	// dummy read to get the Accelerometer into SPI mode
	uint8_t rx_buffer[3] = {0x00, 0x00, 0x00};

	read_address(bmi088_spi, accel_port, accel_pin, tx_buffer, rx_buffer, 3);

	HAL_Delay(1);

	tx_buffer[0] = 0x00;

	read_address(bmi088_spi, accel_port, accel_pin, tx_buffer, rx_buffer, 3);

	if(rx_buffer[2] != 0x1E){
		return 1;
	}
	HAL_Delay(2);

	// dummy setup here
	write_address(bmi088_spi, accel_port, accel_pin, ACCEL_ENABLE_SENSOR_ADDRESS, ACCEL_ENABLE_SENSOR_ON);		// turning on sensor

	HAL_Delay(2);

	write_address(bmi088_spi, accel_port, accel_pin, ACCEL_CONFIG_ADDRESS, ACCEL_CONFIG_ODR_1600_HZ | ACCEL_CONFIG_OVERSAMPLING_OSR4);		// setting up sampling rate and oversampling
	write_address(bmi088_spi, accel_port, accel_pin, ACCEL_RANGE_ADDRESS, ACCEL_RANGE_6G);		// setting up sampling range

	HAL_Delay(2);

	write_address(bmi088_spi, accel_port, accel_pin, ACCEL_POWER_MODE_ADDRESS, ACCEL_POWER_MODE_ACTIVE);		// set to normal power mode
	// need to add real Setup here

	return 0;

}

int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN, bool GYRO_ACCEL_CALIBRATION){
	gyro_accel_calibration = GYRO_ACCEL_CALIBRATION;
	int config_sum = 0;
	config_sum += BMI_INIT_ACCEL(hspi, ACCEL_GPIOx, ACCEL_PIN);
	config_sum += BMI_INIT_GYRO(hspi, GYRO_GPIOx, GYRO_PIN);
	if(config_sum == 0){
		return 0;
	}
	else{
		return 1;
	}
}

void BMI_READ_GYRO_DATA(){
	fucked_accel_reading = true;
	HAL_GPIO_WritePin(accel_port, accel_pin, GPIO_PIN_SET);
	uint8_t tx_buffer[7] = {GYRO_RATE_DATA_ADDRESS, 0, 0, 0, 0, 0, 0};
	uint8_t rx_buffer[7];

	read_address(bmi088_spi, gyro_port, gyro_pin, tx_buffer, rx_buffer, 7);

	gyro_data.gyro_x_raw = ((int16_t)rx_buffer[2] << 8) | rx_buffer[1];
	gyro_data.gyro_y_raw = ((int16_t)rx_buffer[4] << 8) | rx_buffer[3];
	gyro_data.gyro_z_raw = ((int16_t)rx_buffer[6] << 8) | rx_buffer[5];

}

void BMI_READ_ACCEL_DATA(){
	fucked_accel_reading = false;
	uint8_t tx_buffer[8] = {ACCEL_ACCELERATION_DATA_ADDRESS, 0, 0, 0, 0, 0, 0, 0};
	uint8_t rx_buffer[8];

	read_address(bmi088_spi, accel_port, accel_pin, tx_buffer, rx_buffer, 8);

	if(fucked_accel_reading == false){		// check if gyro interrupt happened while accel read
		accel_data.accel_x_raw = ((int16_t)rx_buffer[3] << 8) | rx_buffer[2];
		accel_data.accel_y_raw = ((int16_t)rx_buffer[5] << 8) | rx_buffer[4];
		accel_data.accel_z_raw = ((int16_t)rx_buffer[7] << 8) | rx_buffer[6];

		accel_data.accel_x_mg = ((double)accel_data.accel_x_raw / 32768.0 * 1000.0 * (1 << (1 + 1)) * 1.5);	// replace 4 with (1 << (<0x41>+1) (content of 0x41's register)
		accel_data.accel_y_mg = ((double)accel_data.accel_y_raw / 32768.0 * 1000.0 * (1 << (1 + 1)) * 1.5);
		accel_data.accel_z_mg = ((double)accel_data.accel_z_raw / 32768.0 * 1000.0 * (1 << (1 + 1)) * 1.5);

		double g_force_all_axis = sqrt(pow(accel_data.accel_x_mg, 2) + pow(accel_data.accel_y_mg, 2) + pow(accel_data.accel_z_mg, 2));
		if(g_force_all_axis > 950 && g_force_all_axis < 1050){
			accel_right_for_calibration = true;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		else{
			accel_right_for_calibration = false;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		}
	}

}

void BMI_CALCULATE_ANGLE(uint32_t time_us){		// calculates angles from gyro (integrates each axis) (for best results call after each BMI_READ_GYRO_DATA call)

	gyro_data.angle_x -= BMI_GET_GYRO_X() * ((time_us - last_microseconds) / 1000000.0);
	gyro_data.angle_y -= BMI_GET_GYRO_Y() * ((time_us - last_microseconds) / 1000000.0);
	gyro_data.angle_z -= BMI_GET_GYRO_Z() * ((time_us - last_microseconds) / 1000000.0);

	if(gyro_accel_calibration && accel_right_for_calibration){
		double accel_roll = -atan2f(accel_data.accel_y_mg, accel_data.accel_z_mg) * 180.0f / M_PI;
		double accel_pitch = -atan2f(-accel_data.accel_x_mg, sqrtf(accel_data.accel_y_mg * accel_data.accel_y_mg + accel_data.accel_z_mg * accel_data.accel_z_mg)) * 180.0f / M_PI;
		gyro_data.angle_x += (accel_roll - gyro_data.angle_x) * 0.02;
		gyro_data.angle_y += (accel_pitch - gyro_data.angle_y) * 0.02;
	}

	last_microseconds = time_us;
}

double BMI_GET_GYRO_X(){		// return angular velocity in degrees per second (gyro)
	return (gyro_data.gyro_x_raw / 32767.0) * 2000.0;
}
double BMI_GET_GYRO_Y(){
	return (gyro_data.gyro_y_raw / 32767.0) * 2000.0;
}
double BMI_GET_GYRO_Z(){
	return (gyro_data.gyro_z_raw / 32767.0) * 2000.0;
}

double BMI_GET_GYRO_X_ANGLE(){		// return angle in degrees (gyro + maybe accelerometer)
	return gyro_data.angle_x;
}
double BMI_GET_GYRO_Y_ANGLE(){
	return gyro_data.angle_y;
}
double BMI_GET_GYRO_Z_ANGLE(){
	return gyro_data.angle_z;
}

double BMI_GET_ACCEL_X(){			// return acceleration (accelerometer)
	return accel_data.accel_x_mg;
}
double BMI_GET_ACCEL_Y(){
	return accel_data.accel_y_mg;
}
double BMI_GET_ACCEL_Z(){
	return accel_data.accel_z_mg;
}

void BMI_GYRO_SOFT_RESET(){
	write_address(bmi088_spi, gyro_port, gyro_pin, 0x11, 0xB6);		// IMPORTANT: WAIT FOR AT LEAST 30ms BEFORE COMMUNICATING AGAIN!
}

void BMI_ACCEL_SOFT_RESET(){
	write_address(bmi088_spi, gyro_port, gyro_pin, 0x7E, 0xB6);		// IMPORTANT: WAIT FOR AT LEAST 30ms BEFORE COMMUNICATING AGAIN!
}
