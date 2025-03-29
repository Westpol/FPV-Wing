/*
 * bmp390.c
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 *
 *      TODO: set correct BMP setup (oversampling, iirc filter, etc.)
 */

#include "bmp390.h"

static SPI_HandleTypeDef *bmp390_spi;
static GPIO_TypeDef *baro_port;
static uint16_t baro_pin;

static Baro_Data baro_data = {0};
static Baro_Calibration_Float baro_calibration = {0};

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

static float BMP_COMPENSATE_TEMPERATURE(uint32_t uncomp_temp, Baro_Calibration_Float *calib_data){
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

static float BMP_COMPENSATE_PRESSURE(uint32_t uncomp_press, Baro_Calibration_Float *calib_data){
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

int BMP_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *BARO_GPIOx, uint16_t BARO_PIN){
	bmp390_spi = hspi;
	baro_port = BARO_GPIOx;
	baro_pin = BARO_PIN;

	uint8_t tx_buffer[3] = {0x00, 0x00, 0x00};
	uint8_t rx_buffer[3] = {0x00, 0x00, 0x00};
	read_address(bmp390_spi, baro_port, baro_pin, tx_buffer, rx_buffer, 3);

	if(rx_buffer[2] != 0x60){
		return 1;
	}

	write_address(bmp390_spi, baro_port, baro_pin, POWER_CONTROL_REGISTER, POWER_CONTROL_PRESS_EN_TEMP_EN_NORMAL_MODE);
	write_address(bmp390_spi, baro_port, baro_pin, OVERSAMPLING_REGISTER, OSR_P_16X_T_2X);
	write_address(bmp390_spi, baro_port, baro_pin, OUTPUT_DATA_RATE_REGISTER, OUTPUT_DATA_RATE_25_HZ);
	write_address(bmp390_spi, baro_port, baro_pin, IIR_FILTER_REGISTER, IIR_FILTER_COEF_3);

	uint8_t calib_tx_buffer[23];
	uint8_t calib_rx_buffer[23];
	calib_tx_buffer[0] = 0x31 | READ_BYTE;

	read_address(bmp390_spi, baro_port, baro_pin, calib_tx_buffer, calib_rx_buffer, 22);
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

void BMP_GET_DATA(){		// reads temperature and pressure data and calculates temperature and pressure
	uint8_t tx_buffer[8];
	tx_buffer[0] = 0x04 | READ_BYTE;
	uint8_t rx_buffer[8];

	read_address(bmp390_spi, baro_port, baro_pin, tx_buffer, rx_buffer, 8);
	BMP_CONVERT_DATA(rx_buffer);
}

void BMP_CONVERT_DATA(uint8_t *rx_buffer){
	uint64_t raw_temp = ((uint32_t)*(rx_buffer + 7) << 16) | ((uint32_t)*(rx_buffer + 6) << 8) | *(rx_buffer + 5);
	uint64_t raw_press = ((uint32_t)*(rx_buffer + 4) << 16) | ((uint32_t)*(rx_buffer + 3) << 8) | *(rx_buffer + 2);

	baro_data.temperature = BMP_COMPENSATE_TEMPERATURE(raw_temp, &baro_calibration);
	baro_data.pressure = BMP_COMPENSATE_PRESSURE(raw_press, &baro_calibration);
}


double BMP_GET_HEIGHT(double ground_pressure_pa){		// outputs height in meters compared to pressure at ground level
	return (ground_pressure_pa - baro_data.pressure) / 12.015397;		// calculates height at air temperature of 15°C
}
double BMP_GET_PRESS(){		// outputs pressure in pascal
	return baro_data.pressure;
}
double BMP_GET_TEMP(){		// outputs temperature in celsius
	return baro_data.temperature;
}


void BMP_SOFT_RESET(){		// clears all registers and "resets" the chip
	write_address(bmp390_spi, baro_port, baro_pin, 0x7E, 0xB6);
}

