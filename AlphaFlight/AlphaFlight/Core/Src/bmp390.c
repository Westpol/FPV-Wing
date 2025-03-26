/*
 * bmp390.c
 *
 *  Created on: Mar 21, 2025
 *      Author: benno
 */

#include "bmp390.h"

static SPI_HandleTypeDef *bmp390_spi;
static GPIO_TypeDef *baro_port;
static uint16_t baro_pin;

static Baro_Data baro_data = {0};
static Baro_Calibration baro_calibration = {0};

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

static void BMP_COMPENSATE_TEMPERATURE(){
	uint64_t partial_data1;
	uint64_t partial_data2;
	uint64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;

	/* calculate compensate temperature */
	partial_data1 = (uint64_t)(baro_calibration.raw_temp - (256 * (uint64_t)(baro_calibration.NVM_PAR_T1)));
	partial_data2 = (uint64_t)(baro_calibration.NVM_PAR_T2 * partial_data1);
	partial_data3 = (uint64_t)(partial_data1 * partial_data1);
	partial_data4 = (int64_t)(((int64_t)partial_data3) * ((int64_t)baro_calibration.NVM_PAR_T3));
	partial_data5 = ((int64_t)(((int64_t)partial_data2) * 262144) + (int64_t)partial_data4);
	partial_data6 = (int64_t)(((int64_t)partial_data5) / 4294967296U);
	baro_calibration.t_fine = partial_data6;
	baro_data.temperature = (int64_t)((partial_data6 * 25)  / 16384) / 100.0;
	baro_calibration.temperature = (int64_t)((partial_data6 * 25)  / 16384);
}

static void BMP_COMPENSATE_PRESSURE(){
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t offset;
	int64_t sensitivity;
	uint64_t comp_press;

	/* calculate compensate pressure */
	partial_data1 = baro_calibration.t_fine * baro_calibration.t_fine;
	partial_data2 = partial_data1 / 64;
	partial_data3 = (partial_data2 * baro_calibration.t_fine) / 256;
	partial_data4 = (baro_calibration.NVM_PAR_P8 * partial_data3) / 32;
	partial_data5 = (baro_calibration.NVM_PAR_P7 * partial_data1) * 16;
	partial_data6 = (baro_calibration.NVM_PAR_P6 * baro_calibration.t_fine) * 4194304;
	offset = (int64_t)((int64_t)(baro_calibration.NVM_PAR_P5) * (int64_t)140737488355328U) + partial_data4 + partial_data5 + partial_data6;
	partial_data2 = (((int64_t)baro_calibration.NVM_PAR_P4) * partial_data3) / 32;
	partial_data4 = (baro_calibration.NVM_PAR_P3 * partial_data1) * 4;
	partial_data5 = ((int64_t)(baro_calibration.NVM_PAR_P2) - 16384) * ((int64_t)baro_calibration.t_fine) * 2097152;
	sensitivity = (((int64_t)(baro_calibration.NVM_PAR_P1) - 16384) * (int64_t)70368744177664U) + partial_data2 + partial_data4 + partial_data5;
	partial_data1 = (sensitivity / 16777216) * baro_calibration.raw_press;
	partial_data2 = (int64_t)(baro_calibration.NVM_PAR_P10) * (int64_t)(baro_calibration.t_fine);
	partial_data3 = partial_data2 + (65536 * (int64_t)(baro_calibration.NVM_PAR_P9));
	partial_data4 = (partial_data3 * baro_calibration.raw_press) / 8192;
	partial_data5 = (partial_data4 * baro_calibration.raw_press) / 512;
	partial_data6 = (int64_t)((uint64_t)baro_calibration.raw_press * (uint64_t)baro_calibration.raw_press);
	partial_data2 = ((int64_t)(baro_calibration.NVM_PAR_P11) * (int64_t)(partial_data6)) / 65536;
	partial_data3 = (partial_data2 * baro_calibration.raw_press) / 128;
	partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3;
	comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776U);

	baro_calibration.pressure = comp_press / 100.0;
	baro_data.pressure = comp_press / 100.0;
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
	write_address(bmp390_spi, baro_port, baro_pin, OVERSAMPLING_REGISTER, OSR_P_16X_OSR_T_4X);
	write_address(bmp390_spi, baro_port, baro_pin, OUTPUT_DATA_RATE_REGISTER, OUTPUT_DATA_RATE_200);
	write_address(bmp390_spi, baro_port, baro_pin, IIR_FILTER_REGISTER, IIR_FILTER_COEF_127);

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

	return 0;
}

void BMP_GET_DATA(){
	uint8_t tx_buffer[8];
	tx_buffer[0] = 0x04 | READ_BYTE;
	uint8_t rx_buffer[8];

	read_address(bmp390_spi, baro_port, baro_pin, tx_buffer, rx_buffer, 8);

	baro_calibration.raw_temp = ((uint32_t)rx_buffer[7] << 16) | ((uint32_t)rx_buffer[6] << 8) | rx_buffer[5];

	BMP_COMPENSATE_TEMPERATURE();
	BMP_COMPENSATE_PRESSURE();

}

double BMP_GET_HEIGHT(){
	return 0;
}
double BMP_GET_PRESS(){
	return baro_data.pressure;
}
double BMP_GET_TEMP(){
	return baro_data.temperature;
}

void BMP_SOFT_RESET(){
	write_address(bmp390_spi, baro_port, baro_pin, 0x7E, 0xB6);
}

