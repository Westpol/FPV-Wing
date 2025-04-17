/*
 * servo.c
 *
 *  Created on: Mar 22, 2025
 *      Author: benno
 */

#include "servo.h"

#define MAX_SERVOS 8

static Servo servo[MAX_SERVOS] = {0};
static uint8_t num_active_servos = 0;

int SERVO_ADD(TIM_HandleTypeDef *SERVO_TIMER, uint32_t TIMER_CHANNEL){
	if(num_active_servos < MAX_SERVOS){
		servo[num_active_servos].servo_timer = SERVO_TIMER;
		servo[num_active_servos].timer_channel = TIMER_CHANNEL;
		servo[num_active_servos].servo_microseconds = 1500;
		num_active_servos += 1;
		return num_active_servos - 1;
	}
	else{
		return -1;
	}
}

int SERVO_SET(uint8_t SERVO_NUM, uint16_t TIME_US){
	if(SERVO_NUM < MAX_SERVOS && TIME_US >= 1000 && TIME_US <= 2000){
		servo[SERVO_NUM].servo_microseconds = TIME_US;
		__HAL_TIM_SET_COMPARE(servo[SERVO_NUM].servo_timer, servo[SERVO_NUM].timer_channel, servo[SERVO_NUM].servo_microseconds - 1);
		return 0;
	}
	else{
		return -1;
	}
}

void SERVOS_START_TRANSMISSION(){
	for(int i = 0; i < num_active_servos; i++){
		HAL_TIM_PWM_Start(servo[i].servo_timer, servo[i].timer_channel);
	}
	for(int i = 0; i < num_active_servos; i++){
		__HAL_TIM_SET_COMPARE(servo[i].servo_timer, servo[i].timer_channel, servo[i].servo_microseconds - 1);
	}
}
