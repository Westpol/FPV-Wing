/*
 * bmi088.h
 *
 *  Created on: Mar 19, 2025
 *      Author: benno
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f7xx_hal.h"

typedef struct {
    uint16_t gyro_x_raw;
    uint16_t gyro_y_raw;
    uint16_t gyro_z_raw;
} Gyro_Data;

extern Gyro_Data gyro_data;

int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint8_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint8_t ACCEL_PIN);
void BMI_READ_GYRO_DATA();

void BMI_GET_GYRO_X();
void BMI_GET_GYRO_Y();
void BMI_GET_GYRO_Z();

void BMI_GET_X_RAW();
void BMI_GET_Y_RAW();
void BMI_GET_Z_RAW();


#endif /* INC_BMI088_H_ */
