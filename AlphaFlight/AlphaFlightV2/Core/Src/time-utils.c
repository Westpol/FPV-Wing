/*
 * time-utils.c
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#include "time-utils.h"

TIM_HandleTypeDef *htim;

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx){
	htim = HTIMx;
	HAL_TIM_Base_Start(htim);
}

uint32_t MICROS(){
	return htim->Instance->CNT;
}
