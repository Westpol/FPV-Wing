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
	TIM_HandleTypeDef *servo_timer;
	uint32_t timer_channel;
	uint16_t servo_microseconds;
}Servo;

int SERVO_ADD(TIM_HandleTypeDef *SERVO_TIMER, uint32_t TIMER_CHANNEL);
int SERVO_SET(uint8_t SERVO_NUM, uint16_t TIME_US);
void SERVOS_START_TRANSMISSION(void);

#endif /* INC_SERVO_H_ */
