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
#include "scheduler.h"

static SPI_TypeDef *sensor_spi;
static GPIO_TypeDef *gyro_cs_port;
static uint16_t gyro_cs_pin;
static GPIO_TypeDef *accel_cs_port;
static uint16_t accel_cs_pin;
static GPIO_TypeDef *baro_cs_port;
static uint16_t baro_cs_pin;

static bool accel_right_for_calibration = false;

static Baro_Calibration baro_calibration = {0};
static Sensor_Data sensor_data = {0};
static Raw_Data raw_data = {0};
static uint32_t last_integration_us = 0;

static uint8_t gyro_rx[6] = {0};
static uint8_t accel_rx[7] = {0};
static uint8_t baro_rx[7] = {0};

static void read_address(GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t reg, uint8_t *rxbuffer, uint8_t readLength){
	reg |= READ_BYTE;
	DEVICE_GPIOx->BSRR = (DEVICE_PIN << 16);
	while(!LL_SPI_IsActiveFlag_TXE(sensor_spi));
	LL_SPI_TransmitData8(sensor_spi, reg);

	while(!LL_SPI_IsActiveFlag_RXNE(sensor_spi));
	(void)LL_SPI_ReceiveData8(sensor_spi);  // Clear RXNE from the address byte

	for(int i = 0; i < readLength; i++){
		while(!LL_SPI_IsActiveFlag_TXE(sensor_spi));
		LL_SPI_TransmitData8(sensor_spi, 0x00);
		while(!LL_SPI_IsActiveFlag_RXNE(sensor_spi));
		rxbuffer[i] = LL_SPI_ReceiveData8(sensor_spi);
	}
	while(LL_SPI_IsActiveFlag_BSY(sensor_spi));
	DEVICE_GPIOx->BSRR = (DEVICE_PIN);
}

static void write_address(GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t reg, uint8_t data){
	DEVICE_GPIOx->BSRR = (DEVICE_PIN << 16);
	while (!LL_SPI_IsActiveFlag_TXE(sensor_spi));
	LL_SPI_TransmitData8(sensor_spi, reg);

	while(!LL_SPI_IsActiveFlag_RXNE(sensor_spi));
	(void)LL_SPI_ReceiveData8(sensor_spi);  // Clear RXNE

	while (!LL_SPI_IsActiveFlag_TXE(sensor_spi));
	LL_SPI_TransmitData8(sensor_spi, data);

	while(!LL_SPI_IsActiveFlag_RXNE(sensor_spi));
	(void)LL_SPI_ReceiveData8(sensor_spi);  // Clear RXNE

	while (LL_SPI_IsActiveFlag_BSY(sensor_spi));
	DEVICE_GPIOx->BSRR = (DEVICE_PIN);
}

static int8_t BMI_GYRO_INIT_DATA_READY_PIN_ENABLED(){
	uint8_t rx_buffer = 0;

	read_address(gyro_cs_port, gyro_cs_pin, 0x00, &rx_buffer, 1);		// reads chip ID, if chip doesn't correctly return ID, return error code

	if(rx_buffer != 0x0F){
		return 1;
	}

	// dummy setup here
	write_address(gyro_cs_port, gyro_cs_pin, GYRO_RANGE_ADDRESS, GYRO_RANGE_2000_DEG_PER_SECOND);		// setting gyro range
	write_address(gyro_cs_port, gyro_cs_pin, GYRO_ODR_FILTER_ADDRESS, GYRO_ODR_100_HZ_FILTER_12_HZ);	// setting up filter
	write_address(gyro_cs_port, gyro_cs_pin, GYRO_POWER_MODE_ADDRESS, GYRO_POWER_MODE_NORMAL);		// setting up power mode
	write_address(gyro_cs_port, gyro_cs_pin, 0x15, 0x80);		// enabling interrupt
	write_address(gyro_cs_port, gyro_cs_pin, 0x16, 0b00001011);		// INT4 IO Config
	write_address(gyro_cs_port, gyro_cs_pin, 0x18, 0x80);		// data ready interrupt mapped to INT4
	// need to add real Setup here
	return 0;
}

static int8_t BMI_ACCEL_INIT(){	// dummy read to get the Accelerometer into SPI mode
	uint8_t rx_buffer[2] = {0};

	read_address(accel_cs_port, accel_cs_pin, 0x00, rx_buffer, 2);

	HAL_Delay(1);

	read_address(accel_cs_port, accel_cs_pin, 0x00, rx_buffer, 2);

	if(rx_buffer[1] != 0x1E){
		return 1;
	}
	HAL_Delay(2);

	// dummy setup here
	write_address(accel_cs_port, accel_cs_pin, ACCEL_ENABLE_SENSOR_ADDRESS, ACCEL_ENABLE_SENSOR_ON);		// turning on sensor
	HAL_Delay(2);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_CONFIG_ADDRESS, ACCEL_CONFIG_ODR_1600_HZ | ACCEL_CONFIG_OVERSAMPLING_OSR4);		// setting up sampling rate and oversampling
	write_address(accel_cs_port, accel_cs_pin, ACCEL_RANGE_ADDRESS, ACCEL_RANGE_6G);		// setting up sampling range
	HAL_Delay(2);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_POWER_MODE_ADDRESS, ACCEL_POWER_MODE_ACTIVE);		// set to normal power mode
	// need to add real Setup here

	return 0;
}

static int8_t BMP_BARO_INIT(){

	uint8_t rx_buffer[2] = {0};
	read_address(baro_cs_port, baro_cs_pin, 0x00, rx_buffer, 2);

	if(rx_buffer[1] != 0x60){
		return 1;
	}

	write_address(baro_cs_port, baro_cs_pin, POWER_CONTROL_REGISTER, POWER_CONTROL_PRESS_EN_TEMP_EN_NORMAL_MODE);
	write_address(baro_cs_port, baro_cs_pin, OVERSAMPLING_REGISTER, OSR_P_16X_T_2X);
	write_address(baro_cs_port, baro_cs_pin, OUTPUT_DATA_RATE_REGISTER, OUTPUT_DATA_RATE_25_HZ);
	write_address(baro_cs_port, baro_cs_pin, IIR_FILTER_REGISTER, IIR_FILTER_COEF_3);

	uint8_t calib_rx_buffer[22];

	read_address(baro_cs_port, baro_cs_pin, 0x31, calib_rx_buffer, 22);
	baro_calibration.NVM_PAR_T1 = ((uint16_t)calib_rx_buffer[2] << 8) | calib_rx_buffer[1];
	baro_calibration.NVM_PAR_T2 = ((uint16_t)calib_rx_buffer[4] << 8) | calib_rx_buffer[3];
	baro_calibration.NVM_PAR_T3 = (int8_t)calib_rx_buffer[5];
	baro_calibration.NVM_PAR_P1 = (int16_t)((uint16_t)calib_rx_buffer[7] << 8) | calib_rx_buffer[6];
	baro_calibration.NVM_PAR_P2 = (int16_t)((uint16_t)calib_rx_buffer[9] << 8) | calib_rx_buffer[8];
	baro_calibration.NVM_PAR_P3 = (int8_t)calib_rx_buffer[10];
	baro_calibration.NVM_PAR_P4 = (int8_t)calib_rx_buffer[11];
	baro_calibration.NVM_PAR_P5 = ((uint16_t)calib_rx_buffer[13] << 8) | calib_rx_buffer[12];
	baro_calibration.NVM_PAR_P6 = ((uint16_t)calib_rx_buffer[15] << 8) | calib_rx_buffer[14];
	baro_calibration.NVM_PAR_P7 = (int8_t)calib_rx_buffer[16];
	baro_calibration.NVM_PAR_P8 = (int8_t)calib_rx_buffer[17];
	baro_calibration.NVM_PAR_P9 = (int16_t)((uint16_t)calib_rx_buffer[19] << 8) | calib_rx_buffer[18];
	baro_calibration.NVM_PAR_P10 = (int8_t)calib_rx_buffer[20];
	baro_calibration.NVM_PAR_P11 = (int8_t)calib_rx_buffer[21];

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

int8_t SENSORS_INIT(SPI_TypeDef *HSPIx, GPIO_TypeDef *GYRO_PORT, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_PORT, uint16_t ACCEL_PIN, GPIO_TypeDef *BARO_PORT, uint16_t BARO_PIN){
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

static void GYRO_CONVERT_DATA(){
	raw_data.gyro_x_raw = ((int16_t)gyro_rx[1] << 8) | gyro_rx[0];
	raw_data.gyro_y_raw = ((int16_t)gyro_rx[3] << 8) | gyro_rx[2];
	raw_data.gyro_z_raw = ((int16_t)gyro_rx[5] << 8) | gyro_rx[4];
	sensor_data.gyro_x = (raw_data.gyro_x_raw / 32767.0) * 2000.0;
	sensor_data.gyro_y = -(raw_data.gyro_y_raw / 32767.0) * 2000.0;
	sensor_data.gyro_z = (raw_data.gyro_z_raw / 32767.0) * 2000.0;
}

static void ACCEL_CONVERT_DATA(){
	raw_data.accel_x_raw = ((int16_t)accel_rx[2] << 8) | accel_rx[1];
	raw_data.accel_y_raw = ((int16_t)accel_rx[4] << 8) | accel_rx[3];
	raw_data.accel_z_raw = ((int16_t)accel_rx[6] << 8) | accel_rx[5];
	sensor_data.accel_x = (float)raw_data.accel_x_raw / 32768 * 1000 * 4 * 1.5;
	sensor_data.accel_y = (float)raw_data.accel_y_raw / 32768 * 1000 * 4 * 1.5;
	sensor_data.accel_z = (float)raw_data.accel_z_raw / 32768 * 1000 * 4 * 1.5;

	float overall_force = sqrtf(sensor_data.accel_x * sensor_data.accel_x + sensor_data.accel_y * sensor_data.accel_y + sensor_data.accel_z * sensor_data.accel_z);

	if(overall_force >= 950 && overall_force <= 1050){
		STATUS_LED_GREEN_ON();
		accel_right_for_calibration = true;
	}
	else{
		STATUS_LED_GREEN_OFF();
		accel_right_for_calibration = false;
	}
}

static void BARO_CONVERT_DATA(){
	raw_data.baro_temp_raw = ((uint32_t)baro_rx[6] << 16) | ((uint32_t)baro_rx[5] << 8) | baro_rx[4];
	raw_data.baro_pressure_raw = ((uint32_t)baro_rx[3] << 16) | ((uint32_t)baro_rx[2] << 8) | baro_rx[1];
	sensor_data.temp = BMP_COMPENSATE_TEMPERATURE(raw_data.baro_temp_raw, &baro_calibration);
	sensor_data.pressure = BMP_COMPENSATE_PRESSURE(raw_data.baro_pressure_raw, &baro_calibration);
}

void GYRO_READ(){
	read_address(gyro_cs_port, gyro_cs_pin, 0x02, gyro_rx, 6);
	GYRO_CONVERT_DATA();
}

void ACCEL_READ(){
	read_address(accel_cs_port, accel_cs_pin, 0x12, accel_rx, 7);
	ACCEL_CONVERT_DATA();
}

void BARO_READ(){
	read_address(baro_cs_port, baro_cs_pin, 0x04, baro_rx, 7);
	BARO_CONVERT_DATA();
}

void GYRO_INTEGRATE(void){
	uint32_t now = MICROS();
	uint32_t delta_t = now - last_integration_us;
	last_integration_us = now;
	sensor_data.angle_x_fused += sensor_data.gyro_x * (delta_t / 1000000.0);
	sensor_data.angle_y_fused += sensor_data.gyro_y * (delta_t / 1000000.0);
	sensor_data.angle_z_fused += sensor_data.gyro_z * (delta_t / 1000000.0);
}

void GYRO_FUSION(){
	if(accel_right_for_calibration){
		sensor_data.angle_x_accel = atan2f(sensor_data.accel_y, sensor_data.accel_z) * 180.0f / M_PI;
		sensor_data.angle_y_accel = -atan2f(-sensor_data.accel_x, sqrtf(sensor_data.accel_y * sensor_data.accel_y + sensor_data.accel_z * sensor_data.accel_z)) * 180.0f / M_PI;
		sensor_data.angle_x_fused -= (sensor_data.angle_x_fused - sensor_data.angle_x_accel) * 0.005;
		sensor_data.angle_y_fused -= (sensor_data.angle_y_fused - sensor_data.angle_y_accel) * 0.005;
	}
}

Sensor_Data* SENSOR_DATA_STRUCT(){
	return &sensor_data;
}
