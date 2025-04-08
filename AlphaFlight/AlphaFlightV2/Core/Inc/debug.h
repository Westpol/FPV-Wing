/*
 * debug.h
 *
 *  Created on: Mar 31, 2025
 *      Author: benno
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stdint.h"

void USB_PRINTLN(const char *format, ...);
void USB_PRINT(const char *format, ...);

void STATUS_LED_GREEN_ON();
void STATUS_LED_GREEN_OFF();

#define USB_PRINT_BUFFER_SIZE 128

#endif /* INC_DEBUG_H_ */
