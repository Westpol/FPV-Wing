/*
 * utils.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include "utils.h"
#include <math.h>

void UTIL_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3){		// calculates q1*q2, saves value in q3
	float q_new[4];
	q_new[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_new[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_new[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_new[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	q3[0] = q_new[0]; q3[1] = q_new[1]; q3[2] = q_new[2]; q3[3] = q_new[3];
}

float UTIL_MIN_F(float value, float min){
	if(value <= min){
		return min;
	}
	return value;
}

float UTIL_MAX_F(float value, float max){
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
