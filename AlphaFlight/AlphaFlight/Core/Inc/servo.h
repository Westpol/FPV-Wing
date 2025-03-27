/*
 * servo.h
 *
 *  Created on: Mar 22, 2025
 *      Author: benno
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"

typedef struct{
	GPIO_TypeDef servo_port;
	uint8_t servo_pin;
	uint16_t servo_microseconds;
}Servo;


#endif /* INC_SERVO_H_ */
