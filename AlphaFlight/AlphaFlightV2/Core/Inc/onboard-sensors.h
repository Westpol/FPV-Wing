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

int8_t SENSORS_INIT(SPI_HandleTypeDef *HSPIx);

#endif /* INC_ONBOARD_SENSORS_H_ */
