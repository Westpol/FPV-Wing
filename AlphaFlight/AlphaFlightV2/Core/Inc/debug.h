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
void USB_PRINT_HEX(uint8_t *data, uint32_t len);

void STATUS_LED_GREEN_ON();
void STATUS_LED_GREEN_OFF();
void STATUS_LED_GREEN_TOGGLE();

void STATUS_LED_BLUE_ON();
void STATUS_LED_BLUE_OFF();
void STATUS_LED_BLUE_TOGGLE();

#define USB_PRINT_BUFFER_SIZE 2096

#endif /* INC_DEBUG_H_ */
