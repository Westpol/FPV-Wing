/*
 * utils.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include <math.h>
#include <stdbool.h>

#include "utils.h"
#include "usbd_cdc_if.h"

static bool green_toggle = false;
static bool blue_toggle = false;

extern volatile unsigned char progress_counter;

void UTIL_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3){		// calculates q1*q2, saves value in q3
	float q_new[4];
	q_new[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_new[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_new[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_new[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	q3[0] = q_new[0]; q3[1] = q_new[1]; q3[2] = q_new[2]; q3[3] = q_new[3];
}

float UTIL_MAX_F(float value, float min){
	if(value <= min){
		return min;
	}
	return value;
}

float UTIL_MIN_F(float value, float max){
	if(value >= max){
		return max;
	}
	return value;
}

float UTIL_RADIANS(float degrees){
	return degrees * ((float)M_PI / 180.0f);
}

float UTIL_DEGREES(float radians){
	return radians * (180.0f / (float)M_PI);
}


void UTIL_USB_PRINTLN(const char *format, ...){
    char message[USB_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(message, sizeof(message) - 2, format, args);  // Reserve space for \r\n
    va_end(args);

    // Ensure there's space to append "\r\n"
    if (len > 0 && len < (USB_PRINT_BUFFER_SIZE - 2)) {
        message[len] = '\r';
        message[len + 1] = '\n';
        message[len + 2] = '\0'; // Null-terminate the string
        len += 2;
    }

    CDC_Transmit_FS((uint8_t *)message, len);
}

void UTIL_USB_PRINTLN_BLOCKING(const char *format, ...){
	char message[USB_PRINT_BUFFER_SIZE];
	    va_list args;
	    va_start(args, format);
	    int len = vsnprintf(message, sizeof(message) - 2, format, args);  // Reserve space for \r\n
	    va_end(args);

	    // Ensure there's space to append "\r\n"
	    if (len > 0 && len < (USB_PRINT_BUFFER_SIZE - 2)) {
	        message[len] = '\r';
	        message[len + 1] = '\n';
	        message[len + 2] = '\0'; // Null-terminate the string
	        len += 2;
	    }

	    uint32_t start = HAL_GetTick();
	    while (CDC_Transmit_FS((uint8_t *)message, len) == USBD_BUSY) {
	        if (HAL_GetTick() - start > 100) break; // Timeout after 100ms
	    }
}

void UTIL_USB_PRINT(const char *format, ...) {
    char message[USB_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(message, sizeof(message) - 2, format, args);  // Reserve space for \r\n
    va_end(args);

    // Ensure there's space to append "\r\n"
    if (len > 0 && len < (USB_PRINT_BUFFER_SIZE - 2)) {
        message[len] = '\0';
    }

    CDC_Transmit_FS((uint8_t *)message, len);
}

void UTIL_USB_PRINT_HEX(uint8_t *data, uint32_t len) {
    char buffer[512]; // 3 chars per byte + \r\n + null terminator
    char *ptr = buffer;

    for (uint8_t i = 0; i < len; i++) {
        ptr += sprintf(ptr, "%02X ", data[i]);
    }

    ptr += sprintf(ptr, "\r\n");  // Append \r\n at the end
    CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));
}

void UTIL_STATUS_LED_GREEN_ON(){
	GPIOB->BSRR |= GPIO_PIN_9 << 16;
	green_toggle = true;
}

void UTIL_STATUS_LED_GREEN_OFF(){
	GPIOB->BSRR |= GPIO_PIN_9;
	green_toggle = false;
}

void UTIL_STATUS_LED_GREEN_TOGGLE(){
	if(green_toggle){
		GPIOB->BSRR |= GPIO_PIN_9;
		green_toggle = false;
	}
	else{
		GPIOB->BSRR |= GPIO_PIN_9 << 16;
		green_toggle = true;
	}
}

void UTIL_STATUS_LED_BLUE_ON(){
	GPIOB->BSRR |= GPIO_PIN_8 << 16;
	blue_toggle = true;
}

void UTIL_STATUS_LED_BLUE_OFF(){
	GPIOB->BSRR |= GPIO_PIN_8;
	blue_toggle = false;
}

void UTIL_STATUS_LED_BLUE_TOGGLE(){
	if(blue_toggle){
		GPIOB->BSRR |= GPIO_PIN_8;
		blue_toggle = false;
	}
	else{
		GPIOB->BSRR |= GPIO_PIN_8 << 16;
		blue_toggle = true;
	}
}

static void delay_ms(int delay_ms){
	  for(int i = 0; i < delay_ms; i++){
		  while(!(TIM6->SR & TIM_SR_UIF)){

		  }
		  TIM6->SR &= ~TIM_SR_UIF;
	  }
}

void ERROR_HANDLER_BLINKS(unsigned char BLINKS)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  // Enable TIM6 clock (F7 version)
  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

  // Configure TIM6 for 100 ms overflow (@108 MHz timer clock)
  TIM6->PSC = 1079;     // 108 MHz / 10800 = 10 kHz
  TIM6->ARR = 99;       // 999+1 = 100 ms period
  TIM6->CNT = 0;

  // Enable counter
  TIM6->CR1 |= TIM_CR1_CEN;


  while (1)
  {
	  UTIL_STATUS_LED_GREEN_OFF();
	  UTIL_STATUS_LED_BLUE_OFF();
	  for(uint8_t counter = 0; counter < progress_counter; counter++){
		  UTIL_STATUS_LED_BLUE_ON();
		  delay_ms(100);
		  UTIL_STATUS_LED_BLUE_OFF();
		  delay_ms(500);
	  }

	  delay_ms(800);

	  for(uint8_t counter = 0; counter < BLINKS; counter++){
		  UTIL_STATUS_LED_GREEN_ON();
		  delay_ms(100);
		  UTIL_STATUS_LED_GREEN_OFF();
		  delay_ms(500);
	  }
	  delay_ms(1500);
  }
  /* USER CODE END Error_Handler_Debug */
}
