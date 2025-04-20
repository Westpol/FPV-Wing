/*
 * utils.c
 *
 *  Created on: Apr 19, 2025
 *      Author: benno
 */

#include "utils.h"

float MIN(float value, float min){
	if(value <= min){
		return min;
	}
	return value;
}

float MAX(float value, float max){
	if(value >= max){
		return max;
	}
	return value;
}
