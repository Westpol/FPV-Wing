/*
 * time-utils.c
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#include "time-utils.h"

TIM_HandleTypeDef *htim;
volatile uint32_t timer_high = 0;
volatile uint32_t last_timer_read = 0;

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx) {
	htim = HTIMx;
	HAL_TIM_Base_Start(htim);
}

uint64_t MICROS64(void) {
    uint32_t low   = htim->Instance->CNT;
    if(low < last_timer_read){		// overflow occured
    	timer_high += 1;
    }
    last_timer_read = low;

    return (((uint64_t)timer_high << 32) | low);
}
