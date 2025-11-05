/*
 * utils.h
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#ifndef UTILS_H_
#define UTILS_H_

float UTIL_MAX_F(float value, float min);
float UTIL_MIN_F(float value, float max);
void UTIL_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3);
float UTIL_RADIANS(float degrees);
float UTIL_DEGREES(float radians);

#endif /* UTILS_H_ */
