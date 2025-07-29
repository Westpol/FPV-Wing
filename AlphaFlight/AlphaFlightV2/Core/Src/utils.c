/*
 * utils.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include "utils.h"

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
