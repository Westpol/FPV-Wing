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
#include "time-utils.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

static SPI_TypeDef *sensor_spi;
static GPIO_TypeDef *gyro_cs_port;
static uint16_t gyro_cs_pin;
static GPIO_TypeDef *accel_cs_port;
static uint16_t accel_cs_pin;
static GPIO_TypeDef *baro_cs_port;
static uint16_t baro_cs_pin;

static Baro_Calibration baro_calibration = {0};
IMU_Data imu_data = {0};
static Raw_Data raw_data = {0};
static ALPHA_VALUES alpha_values = {0};
static uint64_t last_integration_us = 0;

static uint8_t gyro_rx[6] = {0};

static uint8_t baro_rx[7] = {0};
uint64_t baro_last_vs_update_time = 0;
float R[3][3] = {
		{1.0f, 0.0f, 0.0f},
		{0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 1.0f}};

__attribute__((optimize("O0"))) static void read_address(GPIO_TypeDef *DEVICE_GPIOx, uint16_t DEVICE_PIN, uint8_t reg, uint8_t *rxbuffer, uint8_t readLength){
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
	write_address(gyro_cs_port, gyro_cs_pin, GYRO_ODR_FILTER_ADDRESS, GYRO_ODR_1000_HZ_FILTER_116_HZ);	// setting up filter
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

	HAL_Delay(10);

	write_address(accel_cs_port, accel_cs_pin, 0x7E, 0xB6);

	HAL_Delay(10);

	read_address(accel_cs_port, accel_cs_pin, 0x00, rx_buffer, 2);

	if(rx_buffer[1] != 0x1E){
		return 1;
	}
	HAL_Delay(10);

	// dummy setup here
	write_address(accel_cs_port, accel_cs_pin, ACCEL_ENABLE_SENSOR_ADDRESS, ACCEL_ENABLE_SENSOR_ON);		// turning on sensor
	HAL_Delay(100);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_CONFIG_ADDRESS, ACCEL_CONFIG_ODR_1600_HZ | ACCEL_CONFIG_OVERSAMPLING_OSR4);		// setting up sampling rate and oversampling
	HAL_Delay(100);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_RANGE_ADDRESS, ACCEL_RANGE_6G);		// setting up sampling range
	HAL_Delay(100);
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

	gyro_cs_port->BSRR = (gyro_cs_pin);
	accel_cs_port->BSRR = (accel_cs_pin);
	baro_cs_port->BSRR = (baro_cs_pin);

	HAL_Delay(10);

	alpha_values.gyro_cutoff_hertz = 50;
	alpha_values.accel_cutoff_hertz = 25;
	alpha_values.gyro_alpha = 1.0f - expf(-2.0f * (float)M_PI * alpha_values.gyro_cutoff_hertz * 0.001);
	alpha_values.accel_alpha = 1.0f - expf(-2.0f * (float)M_PI * alpha_values.accel_cutoff_hertz * 0.004);
	alpha_values.baro_alpha = 0.03;

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
	imu_data.gyro_x = (raw_data.gyro_x_raw / 32767.0) * 2000.0 * (M_PI / 180);
	imu_data.gyro_y = (raw_data.gyro_y_raw / 32767.0) * 2000.0 * (M_PI / 180);
	imu_data.gyro_z = (raw_data.gyro_z_raw / 32767.0) * 2000.0 * (M_PI / 180);
}

static void ACCEL_CONVERT_DATA(uint8_t* accel_rx){
	raw_data.accel_x_raw = ((int16_t)accel_rx[2] << 8) | accel_rx[1];
	raw_data.accel_y_raw = ((int16_t)accel_rx[4] << 8) | accel_rx[3];
	raw_data.accel_z_raw = ((int16_t)accel_rx[6] << 8) | accel_rx[5];
	imu_data.accel_x = (float)raw_data.accel_x_raw / 32768 * 1000 * 4 * 1.5;
	imu_data.accel_y = (float)raw_data.accel_y_raw / 32768 * 1000 * 4 * 1.5;
	imu_data.accel_z = (float)raw_data.accel_z_raw / 32768 * 1000 * 4 * 1.5;

	imu_data.accel_x_filtered = (1.0f - alpha_values.accel_alpha) * imu_data.accel_x_filtered + alpha_values.accel_alpha * imu_data.accel_x;
	imu_data.accel_y_filtered = (1.0f - alpha_values.accel_alpha) * imu_data.accel_y_filtered + alpha_values.accel_alpha * imu_data.accel_y;
	imu_data.accel_z_filtered = (1.0f - alpha_values.accel_alpha) * imu_data.accel_z_filtered + alpha_values.accel_alpha * imu_data.accel_z;
}

#define BARO_PRESSURE_ALPHA 0.3f
static void BARO_CONVERT_DATA(){
	raw_data.baro_temp_raw = ((uint32_t)baro_rx[6] << 16) | ((uint32_t)baro_rx[5] << 8) | baro_rx[4];
	raw_data.baro_pressure_raw = ((uint32_t)baro_rx[3] << 16) | ((uint32_t)baro_rx[2] << 8) | baro_rx[1];
	imu_data.temp = BMP_COMPENSATE_TEMPERATURE(raw_data.baro_temp_raw, &baro_calibration);
	imu_data.pressure = BMP_COMPENSATE_PRESSURE(raw_data.baro_pressure_raw, &baro_calibration);

	imu_data.pressure_filtered = (1.0f - BARO_PRESSURE_ALPHA) * imu_data.pressure_filtered + BARO_PRESSURE_ALPHA * imu_data.pressure;
}

#define vertical_speed_lowpass_alpha 0.3f
static void BARO_CALCULATE_HEIGHT(){
	float last_height = imu_data.height;
	imu_data.height = (imu_data.pressure_base - imu_data.pressure_filtered) / 12.015397;
	uint64_t now = MICROS64();
	float delta_time_seconds = (float)(now - baro_last_vs_update_time) / 1000000.0f;
	baro_last_vs_update_time = now;
	float delta_height_centimeters = (imu_data.height - last_height) * 100.0f;
	if(delta_time_seconds > 0){
		float vertical_speed_cm_s = delta_height_centimeters / delta_time_seconds;
		imu_data.vertical_speed_cm_s = imu_data.vertical_speed_cm_s * (1.0 - vertical_speed_lowpass_alpha) + vertical_speed_cm_s * vertical_speed_lowpass_alpha;
	}
}

void GYRO_READ(){
	read_address(gyro_cs_port, gyro_cs_pin, 0x02, gyro_rx, 6);
	GYRO_CONVERT_DATA();
}

void ACCEL_READ(){
	uint8_t accel_rx[7] = {0};
	read_address(accel_cs_port, accel_cs_pin, 0x12, accel_rx, 7);
	ACCEL_CONVERT_DATA(&accel_rx[0]);
}

void BARO_READ(){
	read_address(baro_cs_port, baro_cs_pin, 0x04, baro_rx, 7);
	BARO_CONVERT_DATA();
	BARO_CALCULATE_HEIGHT();
}

static void MULTIPLY_CUBE(float delta_r[3][3]){
	float R_temp[3][3] = {0};
		for(int i = 0; i < 3; i++){
			for(int k = 0; k < 3; k++){
				for(int j = 0; j < 3; j++){
					R_temp[i][k] += R[i][j] * delta_r[j][k];
				}
			}
		}
		memcpy(R, R_temp, sizeof(R));
}

void GYRO_INTEGRATE(){
	uint64_t now = MICROS64();
	uint64_t delta_t_us = now - last_integration_us;
	last_integration_us = now;
	float delta_t_s = (float)delta_t_us / 1000000.0f;

	float w[3] = {imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z};

	float omega[3][3] = {
			{1.0f, -w[2] * delta_t_s, w[1] * delta_t_s},
			{w[2] * delta_t_s, 1.0f, -w[0] * delta_t_s},
			{-w[1] * delta_t_s, w[0] * delta_t_s, 1.0f}
	};
	MULTIPLY_CUBE(omega);

	imu_data.pitch_angle = -asinf(-R[2][0]) * (180 / M_PI);
	imu_data.roll_angle = atan2f(R[2][1], R[2][2]) * (180 / M_PI);
	imu_data.angle_y_fused = imu_data.pitch_angle;
	imu_data.angle_x_fused = imu_data.roll_angle;
}

#define fusion_alpha 0.02
void GYRO_FUSION(){
	float zb[3] = {R[0][2], R[1][2], R[2][2]};
	float accel[3] = {imu_data.accel_x, imu_data.accel_y, imu_data.accel_z};
	float norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
	accel[0] /= norm;
	accel[1] /= norm;
	accel[2] /= norm;

	float delta[3] = {
	    zb[2]*accel[1] - zb[1]*accel[2],  // roll
	    zb[0]*accel[2] - zb[2]*accel[0],  // pitch
	    0.0f
	};

	float delta_norm = sqrtf(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
	if(delta_norm > 1.0f) delta_norm = 1.0f;
	float theta = asin(delta_norm);

	if(delta_norm > 1e-6f){
		delta[0] /= delta_norm;
		delta[1] /= delta_norm;
		delta[2] /= delta_norm;
	}
	else{
		delta[0] = delta[1] = delta[2] = 0.0f;
		theta = 0.0f;
	}

	theta *= fusion_alpha;

	float c = cosf(theta);
	float s = sinf(theta);
	float t = 1.0f - c;

	float delta_x = delta[0], delta_y = delta[1], delta_z = delta[2];

	float omega[3][3] = {
				{t * delta_x * delta_x + c, t * delta_x * delta_y - s * delta_z, t * delta_x * delta_z + s * delta_y},
			    {t * delta_x * delta_y + s * delta_z, t * delta_y * delta_y + c, t * delta_y * delta_z - s * delta_x},
			    {t * delta_x * delta_z - s * delta_y, t * delta_y * delta_z + s * delta_x, t * delta_z * delta_z + c}
		};
	MULTIPLY_CUBE(omega);
}


static float dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


void GYRO_GRAM_SCHMIDT_NORMALIZE(){
	float u1[3] = {R[0][0], R[0][1], R[0][2]};
	float u2[3] = {R[1][0], R[1][1], R[1][2]};
	float u3[3] = {R[2][0], R[2][1], R[2][2]};

	float v2[3], v3[3];

	// v2 = u2 - proj_u1(u2)
	float proj21 = dot(u2, u1) / dot(u1, u1);
	v2[0] = u2[0] - proj21 * u1[0];
	v2[1] = u2[1] - proj21 * u1[1];
	v2[2] = u2[2] - proj21 * u1[2];

	// v3 = u3 - proj_u1(u3) - proj_u2(u3)
	float proj31 = dot(u3, u1) / dot(u1, u1);
	float proj32 = dot(u3, v2) / dot(v2, v2);
	v3[0] = u3[0] - proj31 * u1[0] - proj32 * v2[0];
	v3[1] = u3[1] - proj31 * u1[1] - proj32 * v2[1];
	v3[2] = u3[2] - proj31 * u1[2] - proj32 * v2[2];


	// normalize
	float norm_u1 = sqrtf(u1[0] * u1[0] + u1[1] * u1[1] + u1[2] * u1[2]);
	float norm_v2 = sqrtf(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
	float norm_v3 = sqrtf(v3[0] * v3[0] + v3[1] * v3[1] + v3[2] * v3[2]);
	u1[0] /= norm_u1; u1[1] /= norm_u1; u1[2] /= norm_u1;
	v2[0] /= norm_v2; v2[1] /= norm_v2; v2[2] /= norm_v2;
	v3[0] /= norm_v3; v3[1] /= norm_v3; v3[2] /= norm_v3;

	for(int i = 0; i < 3; i++){
		R[0][i] = u1[i];
		R[1][i] = v2[i];
		R[2][i] = v3[i];
	}
}


void BARO_SET_BASE_PRESSURE(){
	uint8_t counter = 0;
	while(1){
		BARO_READ();
		if(imu_data.pressure > 0){
			imu_data.pressure_base = imu_data.pressure;
			imu_data.pressure_filtered = imu_data.pressure;
			return;
		}
		if(counter++ > 100){
			ERROR_HANDLER_BLINKS(1);
		}
	}
}

static uint32_t BATTERY_READ_SAMPLE(){
	HAL_ADC_Start(&hadc1);
	    if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
	        return HAL_ADC_GetValue(&hadc1); // 12-bit value: 0–4095
	    }
	    return 0;
	}

#define BAT_NEW_SAMPLE_ALPHA_VALUE 0.01f
#define BAT_VOLTAGE_DIVIDER_SCALING 11.062138728f
void BATTERY_UPDATE(void){
	uint32_t current_battery_value = BATTERY_READ_SAMPLE();
	float voltage = ((float)current_battery_value / 4095.0f) * 3.3f * BAT_VOLTAGE_DIVIDER_SCALING;
	imu_data.vbat = imu_data.vbat * (1.0f - BAT_NEW_SAMPLE_ALPHA_VALUE) + voltage * BAT_NEW_SAMPLE_ALPHA_VALUE;
	imu_data.vbat_raw = current_battery_value;
}
