/*
 * dshot.c
 *
 *  Created on: Jan 6, 2026
 *      Author: benno
 */

#include "dshot.h"

#include <stdbool.h>

#include "stm32f7xx_hal.h"
#include "main.h"

static bool dma_completed = false;

static uint16_t throttle = 0;
static uint16_t frame[17];

static uint8_t calculate_crc(uint16_t dshot_frame){
	uint8_t crc = (~(dshot_frame ^ (dshot_frame >> 4) ^ (dshot_frame >> 8))) & 0x0F;
	return crc;
}

void DSHOT_SET_THROTTLE(uint8_t channel, uint16_t value){
	if(value > 2047){
		value = 0;
	}

	throttle = value;

	if(throttle > 2047){
		throttle = 0;
	}

	throttle = (throttle << 1) | 0;
	throttle = (throttle << 4) | (calculate_crc(throttle) & 0x0F);

	for (int i = 0; i < 16; i++) {
	    if (throttle & (1 << (15 - i))) {
	        frame[i] = DSHOT600_COUNTER_LOW_PULSE_TICKS;
	    }
	    else {
	        frame[i] = DSHOT600_COUNTER_HIGH_PULSE_TICKS;
	    }
	}
	frame[16] = 10000;
}

void DSHOT_SEND_FRAME(){
	TIM1->CCMR1 = (TIM1->CCMR1 & ~ 0x70) | 0x60;	// enable PWM mode

    // 1. Make sure timer is stopped
    __HAL_TIM_DISABLE(&htim1);

    // 2. Reset counter so first bit starts clean
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    TIM1->CR2 &= ~TIM_CR2_OIS1;   // Idle LOW
    TIM1->BDTR |= TIM_BDTR_OSSR | TIM_BDTR_OSSI;

    // 3. Stop DMA in case it was still armed
    HAL_DMA_Abort(&hdma_tim1_up);

    // Force preload registers (ARR, CCR) to latch
    TIM1->EGR = TIM_EGR_UG;

    // 4. Configure DMA transfer
    HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)frame, (uint32_t)&TIM1->CCR1, 17);

    // 5. Enable TIM1 UPDATE DMA request
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

    // 6. Enable timer (this starts the frame)
    __HAL_TIM_ENABLE(&htim1);
}

static void DSHOT_END_TRANSMISSION(void){
	// 1. Disable DMA requests from timer
	__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);

}

void DSHOT_TIMER_INTERRUPT(void){
	if(!dma_completed) return;
	//DSHOT_END_TRANSMISSION();
}

void DSHOT_DMA_TRANSFER_COMPLETED(void){
	//TIM1->CCMR1 = (TIM1->CCMR1 & ~ 0x70) | 0x20;	// disable PWM mode
}
