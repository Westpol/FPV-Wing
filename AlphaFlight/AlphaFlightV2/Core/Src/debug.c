/*
 * debug.c
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */
#include "debug.h"

void USB_PRINT(const char *format, ...) {
    char message[USB_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    CDC_Transmit_FS((uint8_t *)message, strlen(message));
}
