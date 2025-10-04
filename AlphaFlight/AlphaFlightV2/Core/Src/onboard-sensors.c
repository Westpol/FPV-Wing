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
static uint8_t accel_rx[7] = {0};

static uint8_t baro_rx[7] = {0};
uint64_t baro_last_vs_update_time = 0;
float q[4] = {1, 0, 0, 0};

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

static int8_t BMI_ACCEL_INIT(){
	uint8_t rx_buffer[2] = {0};

	read_address(accel_cs_port, accel_cs_pin, 0x00, rx_buffer, 2);

	HAL_Delay(10);

	read_address(accel_cs_port, accel_cs_pin, 0x00, rx_buffer, 2);

	if(rx_buffer[1] != 0x1E){
		return 1;
	}
	HAL_Delay(10);

	// dummy setup here
	write_address(accel_cs_port, accel_cs_pin, ACCEL_POWER_MODE_ADDRESS, ACCEL_POWER_MODE_ACTIVE);		// set to normal power mode
	HAL_Delay(100);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_ENABLE_SENSOR_ADDRESS, ACCEL_ENABLE_SENSOR_ON);		// turning on sensor
	HAL_Delay(10);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_CONFIG_ADDRESS, ACCEL_CONFIG_ODR_1600_HZ | ACCEL_CONFIG_OVERSAMPLING_OSR4);		// setting up sampling rate and oversampling
	HAL_Delay(10);
	write_address(accel_cs_port, accel_cs_pin, ACCEL_RANGE_ADDRESS, ACCEL_RANGE_6G);		// setting up sampling range

	uint8_t status[2] = {0};
	uint32_t counter = 0;
	do {
	    read_address(accel_cs_port, accel_cs_pin, 0x03, status, 2);
	    if(counter++ > 1000){
		    read_address(accel_cs_port, accel_cs_pin, 0x02, status, 2);
	    	DEBUG_PRINT_VERBOSE("Data was not ready");
	    	DEBUG_PRINT_VERBOSE("Error Register: %X", status[1]);
	    	return 1;
	    }
	} while(!(status[1] & 0x80));

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
	DEBUG_PRINT_VERBOSE("INIT Accel");
	return_code_sum += BMI_ACCEL_INIT();
	DEBUG_PRINT_VERBOSE("INIT Baro");
	return_code_sum += BMP_BARO_INIT();
	DEBUG_PRINT_VERBOSE("INIT Gyro");
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

	const float scale = (2000.0f / 32767.0f) * (M_PI / 180.0f);

	float gx_chip = (float)raw_data.gyro_x_raw * scale;
	float gy_chip = (float)raw_data.gyro_y_raw * scale;
	float gz_chip = (float)raw_data.gyro_z_raw * scale;

	imu_data.gyro_x = gx_chip;
	imu_data.gyro_y = -gy_chip;
	imu_data.gyro_z = gz_chip;
}

static void ACCEL_CONVERT_DATA(){
	raw_data.accel_x_raw = ((int16_t)accel_rx[2] << 8) | accel_rx[1];
	raw_data.accel_y_raw = ((int16_t)accel_rx[4] << 8) | accel_rx[3];
	raw_data.accel_z_raw = ((int16_t)accel_rx[6] << 8) | accel_rx[5];
	imu_data.accel_x = -(float)raw_data.accel_x_raw / 32768 * 1000 * 4 * 1.5;
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
	read_address(accel_cs_port, accel_cs_pin, 0x12, accel_rx, 7);
	ACCEL_CONVERT_DATA();
}

void BARO_READ(){
	read_address(baro_cs_port, baro_cs_pin, 0x04, baro_rx, 7);
	BARO_CONVERT_DATA();
	BARO_CALCULATE_HEIGHT();
}

void GYRO_INTEGRATE(){
	uint64_t now = MICROS64();
	uint64_t delta_t_us = now - last_integration_us;
	last_integration_us = now;
	float delta_t_s = (float)delta_t_us / 1000000.0f;

	float wx = imu_data.gyro_x * 0.5f * delta_t_s;
	float wy = imu_data.gyro_y * 0.5f * delta_t_s;
	float wz = imu_data.gyro_z * 0.5f * delta_t_s;

	float w = q[0];
	float x = q[1];
	float y = q[2];
	float z = q[3];

    q[0] += -x*wx - y*wy - z*wz;
    q[1] +=  w*wx + y*wz - z*wy;
    q[2] +=  w*wy - x*wz + z*wx;
    q[3] +=  w*wz + x*wy - y*wx;

    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;

	imu_data.pitch_angle = asin(2*(q[0]*q[2] - q[3]*q[1])) * 180 / M_PI;
	imu_data.roll_angle = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2])) * 180 / M_PI;
	imu_data.angle_y_fused = imu_data.pitch_angle;
	imu_data.angle_x_fused = imu_data.roll_angle;
}


#define FUSION_RATE_HZ 200.0f
#define MAX_DEG_PER_SEC 1.0f
const float correction_angle = (MAX_DEG_PER_SEC * (M_PI/180.0f)) / FUSION_RATE_HZ; // ≈ 8.7266e-5
void GYRO_FUSION(){
	float ax = imu_data.accel_x;
	float ay = imu_data.accel_y;
	float az = imu_data.accel_z;
	float norm = sqrtf(ax*ax + ay*ay + az*az);
	ax /= norm; ay /= norm; az /= norm;

	float vx = 2*(q[1]*q[3] - q[0]*q[2]);
	float vy = 2*(q[0]*q[1] + q[2]*q[3]);
	float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	float ex = (ay*vz - az*vy);
	float ey = (az*vx - ax*vz);
	float ez = (ax*vy - ay*vx);

	float err_norm = sqrtf(ex*ex + ey*ey + ez*ez);
	if(err_norm > 1e-6f){  // avoid division by zero
	    ex /= err_norm;
	    ey /= err_norm;
	    ez /= err_norm;
	}

	float angle = correction_angle;         // tiny rotation
	float half_angle = angle * 0.5f;

	float qs = cosf(half_angle);
	float qx = ex * sinf(half_angle);
	float qy = ey * sinf(half_angle);
	float qz = ez * sinf(half_angle);

	float w = q[0];
	float x = q[1];
	float y = q[2];
	float z = q[3];

	q[0] = qs*w - qx*x - qy*y - qz*z;  // new w
	q[1] = qs*x + qx*w + qy*z - qz*y;  // new x
	q[2] = qs*y - qx*z + qy*w + qz*x;  // new y
	q[3] = qs*z + qx*y - qy*x + qz*w;  // new z

	float norm_q = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] /= norm_q;
	q[1] /= norm_q;
	q[2] /= norm_q;
	q[3] /= norm_q;

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
