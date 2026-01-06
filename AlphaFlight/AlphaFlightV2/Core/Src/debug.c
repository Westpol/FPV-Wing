/*
 * debug.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "debug.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

static bool green_toggle = false;
static bool blue_toggle = false;

extern volatile unsigned char progress_counter;



void STATUS_LED_GREEN_ON(){
	GPIOB->BSRR |= GPIO_PIN_9 << 16;
	green_toggle = true;
}

void STATUS_LED_GREEN_OFF(){
	GPIOB->BSRR |= GPIO_PIN_9;
	green_toggle = false;
}

void STATUS_LED_GREEN_TOGGLE(){
	if(green_toggle){
		GPIOB->BSRR |= GPIO_PIN_9;
		green_toggle = false;
	}
	else{
		GPIOB->BSRR |= GPIO_PIN_9 << 16;
		green_toggle = true;
	}
}

void STATUS_LED_BLUE_ON(){
	GPIOB->BSRR |= GPIO_PIN_8 << 16;
	blue_toggle = true;
}

void STATUS_LED_BLUE_OFF(){
	GPIOB->BSRR |= GPIO_PIN_8;
	blue_toggle = false;
}

void STATUS_LED_BLUE_TOGGLE(){
	if(blue_toggle){
		GPIOB->BSRR |= GPIO_PIN_8;
		blue_toggle = false;
	}
	else{
		GPIOB->BSRR |= GPIO_PIN_8 << 16;
		blue_toggle = true;
	}
}
