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
#include "usbd_cdc_if.h"

void USB_PRINT(const char *format, ...);

#define USB_PRINT_BUFFER_SIZE 128

#endif /* INC_DEBUG_H_ */
