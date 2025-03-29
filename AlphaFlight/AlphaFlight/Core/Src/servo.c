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
static uint8_t current_servo = 0;
static Servo servos_sorted[MAX_SERVOS];
static TIM_HandleTypeDef *htim_used;
static bool transmissionActive = false;
static bool sort_after_transmit = false;

static void sort_servos() {
	if(transmissionActive == true){
		sort_after_transmit = true;
		return;
	}
	for (uint8_t i = 0; i < num_active_servos; i++) {
		servos_sorted[i] = servo[i];
	}

	for (uint8_t i = 1; i < num_active_servos; i++) {
		Servo key = servos_sorted[i];
		int j = i - 1;
		while (j >= 0 && servos_sorted[j].servo_microseconds > key.servo_microseconds) {
			servos_sorted[j + 1] = servos_sorted[j];
			j--;
		}
		servos_sorted[j + 1] = key;
	}
}

int SERVO_ADD(GPIO_TypeDef *servo_port, uint16_t servo_pin){
	if(num_active_servos < MAX_SERVOS){
		servo[num_active_servos].servo_port = servo_port;
		servo[num_active_servos].servo_pin = servo_pin;
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
		sort_after_transmit = true;
		return 0;
	}
	else{
		return -1;
	}
}

int SERVOS_INIT(TIM_HandleTypeDef *HTIMx){
	if (HTIMx == NULL) return -1;

	sort_servos();
	htim_used = HTIMx;
	HAL_TIM_OC_Start_IT(htim_used, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(htim_used);
	HAL_TIM_Base_Start(htim_used);
	return 0;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim != htim_used) return;

	if(current_servo < num_active_servos){
		HAL_GPIO_WritePin(servos_sorted[current_servo - 1].servo_port, servos_sorted[current_servo - 1].servo_pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(htim_used, TIM_CHANNEL_1, servos_sorted[current_servo].servo_microseconds);
		current_servo += 1;
	}
	else{
		HAL_GPIO_WritePin(servos_sorted[current_servo - 1].servo_port, servos_sorted[current_servo - 1].servo_pin, GPIO_PIN_RESET);
		current_servo = 0;
		transmissionActive = false;
		if(sort_after_transmit == true){
			sort_after_transmit = false;
			sort_servos();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim != htim_used) return;

    transmissionActive = true;

	for(int i = 0; i < num_active_servos; i++){
		HAL_GPIO_WritePin(servo[i].servo_port, servo[i].servo_pin, GPIO_PIN_SET);
	}

	__HAL_TIM_SET_COMPARE(htim_used, TIM_CHANNEL_1, servos_sorted[current_servo].servo_microseconds);
	current_servo += 1;
}
