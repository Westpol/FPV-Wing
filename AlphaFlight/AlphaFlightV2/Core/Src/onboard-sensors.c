/*
 * onboard-sensors.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "onboard-sensors.h"
#include "debug.h"
#include "math.h"
#include "stdbool.h"

static SPI_HandleTypeDef *sensor_spi;
static GPIO_TypeDef *gyro_cs_port;
static uint16_t gyro_cs_pin;
static GPIO_TypeDef *accel_cs_port;
static uint16_t accel_cs_pin;
static GPIO_TypeDef *baro_cs_port;
static uint16_t baro_cs_pin;

static bool new_gyro_data = false;
static bool new_accel_data = false;
static bool new_baro_data = false;

static uint8_t gyro_to_accel_counter = 0;
static uint8_t gyro_to_baro_counter = 0;

static Baro_Calibration baro_calibration = {0};
static Sensor_Data sensor_data = {0};
static Raw_Data raw_data = {0};

static uint8_t gyro_rx[7] = {0};
static uint8_t gyro_tx[7] = {0x02 | READ_BYTE, 0, 0, 0, 0, 0, 0};

static uint8_t accel_rx[8] = {0};
static uint8_t accel_tx[8] = {0x12 | READ_BYTE, 0, 0, 0, 0, 0, 0, 0};

static uint8_t baro_rx[8] = {0};
static uint8_t baro_tx[8] = {0x04 | READ_BYTE, 0, 0, 0, 0, 0, 0, 0};

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

static int8_t BMI_GYRO_INIT_DATA_READY_PIN_ENABLED(){
	uint8_t tx_buffer[2] = {0x00, 0x00};
	uint8_t rx_buffer[2] = {0x00, 0x00};

	read_address(sensor_spi, gyro_cs_port, gyro_cs_pin, tx_buffer, rx_buffer, 2);		// reads chip ID, if chip doesn't correctly return ID, return error code

	if(rx_buffer[1] != 0x0F){
		return 1;
	}

	// dummy setup here
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, GYRO_RANGE_ADDRESS, GYRO_RANGE_2000_DEG_PER_SECOND);		// setting gyro range
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, GYRO_ODR_FILTER_ADDRESS, GYRO_ODR_100_HZ_FILTER_12_HZ);	// setting up filter
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, GYRO_POWER_MODE_ADDRESS, GYRO_POWER_MODE_NORMAL);		// setting up power mode
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, 0x15, 0x80);		// enabling interrupt
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, 0x16, 0b00001011);		// INT4 IO Config
	write_address(sensor_spi, gyro_cs_port, gyro_cs_pin, 0x18, 0x80);		// data ready interrupt mapped to INT4
	// need to add real Setup here
	return 0;
}

static int8_t BMI_ACCEL_INIT(){
	uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};	// dummy read to get the Accelerometer into SPI mode
	uint8_t rx_buffer[3] = {0x00, 0x00, 0x00};

	read_address(sensor_spi, accel_cs_port, accel_cs_pin, tx_buffer, rx_buffer, 3);

	HAL_Delay(1);

	tx_buffer[0] = 0x00;

	read_address(sensor_spi, accel_cs_port, accel_cs_pin, tx_buffer, rx_buffer, 3);

	if(rx_buffer[2] != 0x1E){
		return 1;
	}
	HAL_Delay(2);

	// dummy setup here
	write_address(sensor_spi, accel_cs_port, accel_cs_pin, ACCEL_ENABLE_SENSOR_ADDRESS, ACCEL_ENABLE_SENSOR_ON);		// turning on sensor
	HAL_Delay(2);
	write_address(sensor_spi, accel_cs_port, accel_cs_pin, ACCEL_CONFIG_ADDRESS, ACCEL_CONFIG_ODR_1600_HZ | ACCEL_CONFIG_OVERSAMPLING_OSR4);		// setting up sampling rate and oversampling
	write_address(sensor_spi, accel_cs_port, accel_cs_pin, ACCEL_RANGE_ADDRESS, ACCEL_RANGE_6G);		// setting up sampling range
	HAL_Delay(2);
	write_address(sensor_spi, accel_cs_port, accel_cs_pin, ACCEL_POWER_MODE_ADDRESS, ACCEL_POWER_MODE_ACTIVE);		// set to normal power mode
	// need to add real Setup here

	return 0;
}

static int8_t BMP_BARO_INIT(){

	uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};
	uint8_t rx_buffer[3] = {0x00, 0x00, 0x00};
	read_address(sensor_spi, baro_cs_port, baro_cs_pin, tx_buffer, rx_buffer, 3);

	if(rx_buffer[2] != 0x60){
		return 1;
	}

	write_address(sensor_spi, baro_cs_port, baro_cs_pin, POWER_CONTROL_REGISTER, POWER_CONTROL_PRESS_EN_TEMP_EN_NORMAL_MODE);
	write_address(sensor_spi, baro_cs_port, baro_cs_pin, OVERSAMPLING_REGISTER, OSR_P_16X_T_2X);
	write_address(sensor_spi, baro_cs_port, baro_cs_pin, OUTPUT_DATA_RATE_REGISTER, OUTPUT_DATA_RATE_25_HZ);
	write_address(sensor_spi, baro_cs_port, baro_cs_pin, IIR_FILTER_REGISTER, IIR_FILTER_COEF_3);

	uint8_t calib_tx_buffer[23];
	uint8_t calib_rx_buffer[23];
	calib_tx_buffer[0] = 0x31 | READ_BYTE;

	read_address(sensor_spi, baro_cs_port, baro_cs_pin, calib_tx_buffer, calib_rx_buffer, 22);
	baro_calibration.NVM_PAR_T1 = ((uint16_t)calib_rx_buffer[3] << 8) | calib_rx_buffer[2];
	baro_calibration.NVM_PAR_T2 = ((uint16_t)calib_rx_buffer[5] << 8) | calib_rx_buffer[4];
	baro_calibration.NVM_PAR_T3 = (int8_t)calib_rx_buffer[6];
	baro_calibration.NVM_PAR_P1 = (int16_t)((uint16_t)calib_rx_buffer[8] << 8) | calib_rx_buffer[7];
	baro_calibration.NVM_PAR_P2 = (int16_t)((uint16_t)calib_rx_buffer[10] << 8) | calib_rx_buffer[9];
	baro_calibration.NVM_PAR_P3 = (int8_t)calib_rx_buffer[11];
	baro_calibration.NVM_PAR_P4 = (int8_t)calib_rx_buffer[12];
	baro_calibration.NVM_PAR_P5 = ((uint16_t)calib_rx_buffer[14] << 8) | calib_rx_buffer[13];
	baro_calibration.NVM_PAR_P6 = ((uint16_t)calib_rx_buffer[16] << 8) | calib_rx_buffer[15];
	baro_calibration.NVM_PAR_P7 = (int8_t)calib_rx_buffer[17];
	baro_calibration.NVM_PAR_P8 = (int8_t)calib_rx_buffer[18];
	baro_calibration.NVM_PAR_P9 = (int16_t)((uint16_t)calib_rx_buffer[20] << 8) | calib_rx_buffer[19];
	baro_calibration.NVM_PAR_P10 = (int8_t)calib_rx_buffer[21];
	baro_calibration.NVM_PAR_P11 = (int8_t)calib_rx_buffer[22];

	baro_calibration.par_t1 = (float)baro_calibration.NVM_PAR_T1 / pow(2, -8);
	baro_calibration.par_t2 = (float)baro_calibration.NVM_PAR_T2 / pow(2, 30);
	baro_calibration.par_t3 = (float)baro_calibration.NVM_PAR_T3 / pow(2, 48);
	baro_calibration.par_p1 = ((float)baro_calibration.NVM_PAR_P1 - pow(2, 14)) / pow(2, 20);
	baro_calibration.par_p2 = ((float)baro_calibration.NVM_PAR_P2 - pow(2, 14)) / pow(2, 29);
	baro_calibration.par_p3 = (float)baro_calibration.NVM_PAR_P3 / pow(2, 32);
	baro_calibration.par_p4 = (float)baro_calibration.NVM_PAR_P4 / pow(2, 37);
	baro_calibration.par_p5 = (float)baro_calibration.NVM_PAR_P5 / pow(2, -3);
	baro_calibration.par_p6 = (float)baro_calibration.NVM_PAR_P6 / pow(2, 6);
	baro_calibration.par_p7 = (float)baro_calibration.NVM_PAR_P7 / pow(2, 8);
	baro_calibration.par_p8 = (float)baro_calibration.NVM_PAR_P8 / pow(2, 15);
	baro_calibration.par_p9 = (float)baro_calibration.NVM_PAR_P9 / pow(2, 48);
	baro_calibration.par_p10 = (float)baro_calibration.NVM_PAR_P10 / pow(2, 48);
	baro_calibration.par_p11 = (float)baro_calibration.NVM_PAR_P11 / pow(2, 65);

	return 0;
}

static float BMP_COMPENSATE_TEMPERATURE(uint32_t uncomp_temp, Baro_Calibration *calib_data){
	float partial_data1;
	float partial_data2;
	partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
	partial_data2 = (float)(partial_data1 * calib_data->par_t2);
	/* Update the compensated temperature in calib structure since this is
	* needed for pressure calculation */
	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
	/* Returns compensated temperature */
	return calib_data->t_lin;
}

static float BMP_COMPENSATE_PRESSURE(uint32_t uncomp_press, Baro_Calibration *calib_data){
	/* Variable to store the compensated pressure */
	float comp_press;
	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;
	/* Calibration data */
	partial_data1 = calib_data->par_p6 * calib_data->t_lin;
	partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = calib_data->par_p2 * calib_data->t_lin;
	partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
	comp_press = partial_out1 + partial_out2 + partial_data4;
	return comp_press;
}

static void BYTES_TO_VALUES(){
	if(new_gyro_data){

		new_gyro_data = false;
	}
	if(new_accel_data){

		new_accel_data = false;
	}
	if(new_baro_data){

		new_baro_data = false;
	}
}

int8_t SENSORS_INIT(SPI_HandleTypeDef *HSPIx, GPIO_TypeDef *GYRO_PORT, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_PORT, uint16_t ACCEL_PIN, GPIO_TypeDef *BARO_PORT, uint16_t BARO_PIN){
	sensor_spi = HSPIx;
	gyro_cs_port = GYRO_PORT;
	gyro_cs_pin = GYRO_PIN;
	accel_cs_port = ACCEL_PORT;
	accel_cs_pin = ACCEL_PIN;
	baro_cs_port = BARO_PORT;
	baro_cs_pin = BARO_PIN;

	int16_t return_code_sum = 0;
	return_code_sum += BMI_ACCEL_INIT();
	return_code_sum += BMP_BARO_INIT();
	return_code_sum += BMI_GYRO_INIT_DATA_READY_PIN_ENABLED();
	if(return_code_sum == 0){
		return 0;
	}
	else{
		return 1;
	}
}

void SENSORS_READ(){
	gyro_to_accel_counter++;
	gyro_to_baro_counter++;
	new_gyro_data = true;
	HAL_GPIO_WritePin(gyro_cs_port, gyro_cs_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(sensor_spi, gyro_tx, gyro_rx, 7);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspix){
	if(hspix->Instance != sensor_spi->Instance)return;
	BYTES_TO_VALUES();
	HAL_GPIO_WritePin(gyro_cs_port, gyro_cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(accel_cs_port, accel_cs_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(baro_cs_port, baro_cs_pin, GPIO_PIN_SET);
	if(gyro_to_accel_counter >= GYRO_PER_ACCEL_READ){
		new_accel_data = true;
		gyro_to_accel_counter = 0;
		HAL_GPIO_WritePin(accel_cs_port, accel_cs_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_DMA(sensor_spi, accel_tx, accel_rx, 8);
	}
	if(gyro_to_baro_counter >= GYRO_PER_BARO_READ){
		new_baro_data = true;
		gyro_to_baro_counter = 0;
		HAL_GPIO_WritePin(baro_cs_port, baro_cs_pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive_DMA(sensor_spi, baro_tx, baro_rx, 8);
	}
}
