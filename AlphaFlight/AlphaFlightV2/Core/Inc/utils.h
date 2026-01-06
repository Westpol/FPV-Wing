/*
 * utils.h
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

float UTIL_MAX_F(float value, float min);
float UTIL_MIN_F(float value, float max);
void UTIL_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3);
float UTIL_RADIANS(float degrees);
float UTIL_DEGREES(float radians);

void UTIL_USB_PRINTLN(const char *format, ...);
void UTIL_USB_PRINTLN_BLOCKING(const char *format, ...);
void UTIL_USB_PRINT(const char *format, ...);
void UTIL_USB_PRINT_HEX(uint8_t *data, uint32_t len);
void UTIL_USB_PRINT_RAW(const char* message, uint32_t len);


#define USB_PRINT_BUFFER_SIZE 2096

#endif /* UTILS_H_ */
