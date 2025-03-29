/*
 * bmi088.h
 *
 *  Created on: Mar 19, 2025
 *      Author: benno
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f7xx_hal.h"
#include "stdbool.h"
#include "math.h"

typedef struct {		// all gyro data is stored in here
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    double angle_x;
    double angle_y;
    double angle_z;
} Gyro_Data;

typedef struct {		// all accelerometer data is stored in here
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    double accel_x_mg;
    double accel_y_mg;
    double accel_z_mg;
} Accel_Data;

int BMI_INIT_GYRO(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN);
int BMI_INIT_ACCEL(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN);
int BMI_INIT(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GYRO_GPIOx, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_GPIOx, uint16_t ACCEL_PIN, bool GYRO_ACCEL_CALIBRATION);		// initializes the whole chip (calls BMI_INIT_GYRO and then BMI_INIT_ACCEL)
void BMI_READ_GYRO_DATA();		// reads from gyro registers
void BMI_CONVERT_GYRO_DATA(uint8_t *rx_buffer);
void BMI_READ_ACCEL_DATA();		// reads from accel registers
void BMI_CONVERT_ACCEL_DATA(uint8_t *rx_buffer);
void BMI_CALCULATE_ANGLE(uint32_t time_us);		// calculates angles from gyro (integrates each axis)

double BMI_GET_GYRO_X();		// returns struct values
double BMI_GET_GYRO_Y();
double BMI_GET_GYRO_Z();

double BMI_GET_GYRO_X_ANGLE();
double BMI_GET_GYRO_Y_ANGLE();
double BMI_GET_GYRO_Z_ANGLE();

double BMI_GET_ACCEL_X();
double BMI_GET_ACCEL_Y();
double BMI_GET_ACCEL_Z();

void BMI_GYRO_SOFT_RESET(void);
void BMI_ACCEL_SOFT_RESET(void);


#define WRITE_BYTE 0x7F		// for write: packet = value & WRITE_BYTE  / for read: packet = value | READ_BYTE
#define READ_BYTE 0x80

#define ACCEL_X_OFFSET 12
#define ACCEL_Y_OFFSET 10
#define ACCEL_Z_OFFSET 0

// -------------------------------------- all gyro setting addresses and values -------------------------------
#define GYRO_RATE_DATA_ADDRESS 0x02

#define GYRO_RANGE_ADDRESS 0x0F
#define GYRO_RANGE_2000_DEG_PER_SECOND 0x00
#define GYRO_RANGE_1000_DEG_PER_SECOND 0x01
#define GYRO_RANGE_500_DEG_PER_SECOND 0x02
#define GYRO_RANGE_250_DEG_PER_SECOND 0x03
#define GYRO_RANGE_125_DEG_PER_SECOND 0x04

#define GYRO_ODR_FILTER_ADDRESS 0x10
#define GYRO_ODR_2000_HZ_FILTER_532_HZ 0x00
#define GYRO_ODR_2000_HZ_FILTER_230_HZ 0x01
#define GYRO_ODR_1000_HZ_FILTER_116_HZ 0x02
#define GYRO_ODR_400_HZ_FILTER_47_HZ 0x03
#define GYRO_ODR_200_HZ_FILTER_23_HZ 0x04
#define GYRO_ODR_100_HZ_FILTER_12_HZ 0x05
#define GYRO_ODR_200_HZ_FILTER_64_HZ 0x06
#define GYRO_ODR_100_HZ_FILTER_32_HZ 0x07

#define GYRO_POWER_MODE_ADDRESS 0x11
#define GYRO_POWER_MODE_NORMAL 0x00
#define GYRO_POWER_MODE_SUSPEND 0x80
// ------------------------------------------------------------------------------------------------------------

// -------------------------------------- all gyro setting addresses and values -------------------------------
#define ACCEL_ACCELERATION_DATA_ADDRESS 0x12

#define ACCEL_CONFIG_ADDRESS 0x40
#define ACCEL_CONFIG_OVERSAMPLING_OSR4 0x08
#define ACCEL_CONFIG_OVERSAMPLING_OSR2 0x09
#define ACCEL_CONFIG_OVERSAMPLING_NORMAL 0x0A
#define ACCEL_CONFIG_ODR_1600_HZ 0x0C
#define ACCEL_CONFIG_ODR_800_HZ 0x0B
#define ACCEL_CONFIG_ODR_400_HZ 0x0A
#define ACCEL_CONFIG_ODR_200_HZ 0x09
#define ACCEL_CONFIG_ODR_100_HZ 0x08
#define ACCEL_CONFIG_ODR_50_HZ 0x07
#define ACCEL_CONFIG_ODR_25_HZ 0x06
#define ACCEL_CONFIG_ODR_12_5_HZ 0x05

#define ACCEL_RANGE_ADDRESS 0x41
#define ACCEL_RANGE_3G 0x00
#define ACCEL_RANGE_6G 0x01
#define ACCEL_RANGE_12G 0x02
#define ACCEL_RANGE_24G 0x03

#define ACCEL_POWER_MODE_ADDRESS 0x7C
#define ACCEL_POWER_MODE_SUSPEND 0x03
#define ACCEL_POWER_MODE_ACTIVE 0x00

#define ACCEL_ENABLE_SENSOR_ADDRESS 0x7D
#define ACCEL_ENABLE_SENSOR_OFF 0x00
#define ACCEL_ENABLE_SENSOR_ON 0x04
// ------------------------------------------------------------------------------------------------------------

#endif /* INC_BMI088_H_ */
