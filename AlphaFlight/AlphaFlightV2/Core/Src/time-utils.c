/*
 * time-utils.c
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#include "time-utils.h"
#include <stdbool.h>

TIM_HandleTypeDef *htim = NULL;
volatile uint32_t timer_high = 0;
volatile uint32_t last_timer_read = 0;
static volatile bool initialized = false;

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx) {
	htim = HTIMx;
	HAL_TIM_Base_Start(htim);
	initialized = true;
}

uint64_t MICROS64(void) {
	if(initialized){
		uint32_t timer_low   = htim->Instance->CNT;
		if(timer_low < last_timer_read){		// overflow occured
			timer_high += 1;
		}
		last_timer_read = timer_low;

		return (((uint64_t)timer_high << 32) | timer_low);
	}
	return 0;
}
