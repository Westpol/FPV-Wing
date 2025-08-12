/*
 * time-utils.c
 *
 *  Created on: Apr 10, 2025
 *      Author: benno
 */

#include "time-utils.h"

TIM_HandleTypeDef *htim;
volatile uint64_t timer_high = 0;

void TIME_UTILS_MICROS_TIM_START(TIM_HandleTypeDef *HTIMx) {
    htim = HTIMx;
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(htim);
}

uint64_t MICROS64(void) {
    uint64_t high1 = timer_high;
    uint32_t low   = htim->Instance->CNT;
    uint64_t high2 = timer_high;

    if (high1 != high2) {
        // Overflow happened between reading high and low → use new high
        low = htim->Instance->CNT;
        high1 = high2;
    }

    return (high1 | low);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim_cb) {
    if (htim_cb == htim) {
        timer_high += ((uint64_t)1 << 32);
    }
}
