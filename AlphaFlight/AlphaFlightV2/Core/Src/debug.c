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
