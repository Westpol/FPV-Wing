/*
 * qmc5883.c
 *
 *  Created on: Apr 25, 2025
 *      Author: benno
 *
 *      Magnetometer on GPS used for navigation
 */


// I2C address: 0x0D
#include "qmc5883.h"
#include "stm32f7xx_hal.h"
#include "debug.h"

#define QMC5883_ADDR (0x0D << 1)  // 7-bit address shifted left for HAL
#define QMC5883_CHIP_ID_REG 0x0D

extern I2C_HandleTypeDef hi2c1;

void MAGNETOMETER_READ_BLOCKING(){
	uint8_t data_buffer[6] = {0};
	int16_t x, y, z;
    if(HAL_I2C_Mem_Read(&hi2c1, QMC5883_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, data_buffer, 6, 100) == HAL_OK){
    	x = (uint16_t)(data_buffer[1] << 8) | data_buffer[0];
    	y = (uint16_t)(data_buffer[3] << 8) | data_buffer[2];
    	z = (uint16_t)(data_buffer[5] << 8) | data_buffer[4];
        USB_PRINTLN_BLOCKING("\n%d %d %d\n", x, y, z);
    }
}

void MAGNETOMETER_INIT(){
	uint8_t data = 0x0D;
	HAL_I2C_Mem_Write(&hi2c1, QMC5883_ADDR, 0x09, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}
