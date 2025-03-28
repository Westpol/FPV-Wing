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
#include "stdbool.h"

typedef struct{
	GPIO_TypeDef *servo_port;
	uint8_t servo_pin;
	uint16_t servo_microseconds;
}Servo;

int SERVOS_INIT(TIM_HandleTypeDef *HTIMx);
int SERVO_ADD(GPIO_TypeDef *servo_port, uint8_t servo_pin);
int SERVO_SET(uint8_t SERVO_NUM, uint16_t TIME_US);


#endif /* INC_SERVO_H_ */
