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
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    double angle_x;
    double angle_y;
    double angle_z;
} Gyro_Data;

extern Gyro_Data gyro_data;

int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN);
void BMI_READ_GYRO_DATA();
void BMI_CALCULATE_ANGLE(uint32_t time_us);

double BMI_GET_GYRO_X();
double BMI_GET_GYRO_Y();
double BMI_GET_GYRO_Z();

double BMI_GET_GYRO_X_ANGLE();
double BMI_GET_GYRO_Y_ANGLE();
double BMI_GET_GYRO_Z_ANGLE();

int16_t BMI_GET_GYRO_X_RAW();
int16_t BMI_GET_GYRO_Y_RAW();
int16_t BMI_GET_GYRO_Z_RAW();

void BMI_GYRO_SOFT_RESET();

#define WRITE_BYTE 0x7F
#define READ_BYTE 0x80

#define GYRO_RANGE_ADRESS 0x0F
#define GYRO_RANGE_2000_DEG_PER_SECOND 0x00
#define GYRO_RANGE_1000_DEG_PER_SECOND 0x01
#define GYRO_RANGE_500_DEG_PER_SECOND 0x02
#define GYRO_RANGE_250_DEG_PER_SECOND 0x03
#define GYRO_RANGE_125_DEG_PER_SECOND 0x04

#define GYRO_ODR_FILTER_ADRESS 0x10
#define GYRO_ODR_2000_HZ_FILTER_532_HZ 0x00
#define GYRO_ODR_2000_HZ_FILTER_230_HZ 0x01
#define GYRO_ODR_1000_HZ_FILTER_116_HZ 0x02
#define GYRO_ODR_400_HZ_FILTER_47_HZ 0x03
#define GYRO_ODR_200_HZ_FILTER_23_HZ 0x04
#define GYRO_ODR_100_HZ_FILTER_12_HZ 0x05
#define GYRO_ODR_200_HZ_FILTER_64_HZ 0x06
#define GYRO_ODR_100_HZ_FILTER_32_HZ 0x07

#define GYRO_POWER_MODE_ADRESS 0x11
#define GYRO_POWER_MODE_NORMAL 0x00
#define GYRO_POWER_MODE_SUSPEND 0x80

#endif /* INC_BMI088_H_ */
