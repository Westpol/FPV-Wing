/*
 * onboard-sensors.h
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */

#ifndef INC_ONBOARD_SENSORS_H_
#define INC_ONBOARD_SENSORS_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"
#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_dma.h"

typedef struct {
	uint16_t NVM_PAR_T1;
	uint16_t NVM_PAR_T2;
	int8_t NVM_PAR_T3;
	int16_t NVM_PAR_P1;
	int16_t NVM_PAR_P2;
	int8_t NVM_PAR_P3;
	int8_t NVM_PAR_P4;
	uint16_t NVM_PAR_P5;
	uint16_t NVM_PAR_P6;
	int8_t NVM_PAR_P7;
	int8_t NVM_PAR_P8;
	int16_t NVM_PAR_P9;
	int8_t NVM_PAR_P10;
	int8_t NVM_PAR_P11;
	float par_t1;
	float par_t2;
	float par_t3;
	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;
	float t_lin;
} Baro_Calibration;

typedef struct{
	float gyro_alpha;
	float accel_alpha;
	float baro_alpha;
	float gyro_cutoff_hertz;
	float accel_cutoff_hertz;
}ALPHA_VALUES;

typedef struct{
	float gyro_x;		// rotational velocity in °/s
	float gyro_y;
	float gyro_z;
	float gyro_x_filtered;		// rotational velocity in °/s with lowpass filter
	float gyro_y_filtered;
	float gyro_z_filtered;
	float angle_x_fused;		// angle in °
	float angle_y_fused;
	float angle_z_fused;
	float angle_x_accel;
	float angle_y_accel;
	float accel_x;		// acceleration in mg (9.81mm/s²)
	float accel_y;
	float accel_z;
	float accel_x_filtered;		// acceleration in mg (9.81mm/s²) with lowpass filter
	float accel_y_filtered;
	float accel_z_filtered;
	float temp;
	float pressure;
	float pressure_filtered;
	float pressure_base;
	float height;
}IMU_Data;

typedef struct{
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
	int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    uint32_t baro_temp_raw;
    uint32_t baro_pressure_raw;
}Raw_Data;

int8_t SENSORS_INIT(SPI_TypeDef *HSPIx, GPIO_TypeDef *GYRO_PORT, uint16_t GYRO_PIN, GPIO_TypeDef *ACCEL_PORT, uint16_t ACCEL_PIN, GPIO_TypeDef *BARO_PORT, uint16_t BARO_PIN);

void SENSORS_READ(void);
void GYRO_READ(void);
void ACCEL_READ(void);
void BARO_READ(void);
void GYRO_INTEGRATE(void);
void GYRO_FUSION(void);
void BARO_SET_BASE_PRESSURE();

#define GYRO_PER_ACCEL_READ 2
#define GYRO_PER_BARO_READ 10

#define WRITE_BYTE 0x7F		// for write: packet = value & WRITE_BYTE  / for read: packet = value | READ_BYTE
#define READ_BYTE 0x80

// -------------------------------------- BMI088 Accel register defines -------------------------------
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

// -------------------------------------- BMI088 Accel register defines -------------------------------
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

//---------------------------------------- BMP390 register defines -------------------------------------------------
#define PRESSURE_DATA_REGISTER 0x04
#define TEMPERATURE_DATA_REGISTER 0x07

#define POWER_CONTROL_REGISTER 0x1B
#define POWER_CONTROL_PRESS_EN_TEMP_EN_NORMAL_MODE 0b00110011

#define OVERSAMPLING_REGISTER 0x1C
#define OSR_P_16X_T_2X 0b00001100

#define OUTPUT_DATA_RATE_REGISTER 0x1D
#define OUTPUT_DATA_RATE_25_HZ 0x03

#define IIR_FILTER_REGISTER 0x1F
#define IIR_FILTER_COEF_3 0b00000100
//-----------------------------------------------------------------------------------------------------------

#endif /* INC_ONBOARD_SENSORS_H_ */
