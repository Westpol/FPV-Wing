/*
 * debug.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "debug.h"
#include "usbd_cdc_if.h"

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

void STATUS_LED_GREEN_ON(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void STATUS_LED_GREEN_OFF(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}
