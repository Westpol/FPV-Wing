/*
 * debug.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "debug.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"

static bool green_toggle = false;
static bool blue_toggle = false;

extern volatile unsigned char progress_counter;

void USB_PRINTLN(const char *format, ...) {
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

void USB_PRINT(const char *format, ...) {
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

void USB_PRINT_HEX(uint8_t *data, uint32_t len) {
    char buffer[512]; // 3 chars per byte + \r\n + null terminator
    char *ptr = buffer;

    for (uint8_t i = 0; i < len; i++) {
        ptr += sprintf(ptr, "%02X ", data[i]);
    }

    ptr += sprintf(ptr, "\r\n");  // Append \r\n at the end
    CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));
}

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

static __attribute__((noinline)) void BAREBONES_DELAY_CYCLES(uint32_t cycles) {
    __asm volatile (
        "1: \n"
        "   subs %0, #1 \n"  // 1 cycle
        "   bne 1b \n"       // 1-2 cycles depending on pipeline
        : "+r" (cycles)
    );
}

static void BAREBONES_DELAY_MS(uint32_t ms){
    while (ms--) {
        BAREBONES_DELAY_CYCLES(108000);	// about how many cycles per milliseconds
    }
}

void ERROR_HANDLER_BLINKS(unsigned char BLINKS)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  STATUS_LED_GREEN_OFF();
	  STATUS_LED_BLUE_OFF();
	  for(uint8_t counter = 0; counter < progress_counter; counter++){
		  STATUS_LED_BLUE_ON();
		  BAREBONES_DELAY_MS(200);
		  STATUS_LED_BLUE_OFF();
		  BAREBONES_DELAY_MS(500);
	  }

	  BAREBONES_DELAY_MS(800);

	  for(uint8_t counter = 0; counter < BLINKS; counter++){
		  STATUS_LED_GREEN_ON();
		  BAREBONES_DELAY_MS(200);
		  STATUS_LED_GREEN_OFF();
		  BAREBONES_DELAY_MS(500);
	  }
	  BAREBONES_DELAY_MS(1500);
  }
  /* USER CODE END Error_Handler_Debug */
}
