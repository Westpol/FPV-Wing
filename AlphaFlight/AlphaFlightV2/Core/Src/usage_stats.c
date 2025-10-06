/*
 * usage_stats.c
 *
 *  Created on: Oct 4, 2025
 *      Author: benno
 */

#include "usage_stats.h"
#include "time-utils.h"

static uint64_t start_time_us = 0;
static float avrg_1s;
static uint32_t max_loop_time_global = 0;
static uint32_t max_loop_time = 0;
static uint32_t max_loop_time_1s = 0;
static uint64_t second_reset_counter = 1000000;
static uint64_t scheduler_not_finished = 0;
static uint64_t scheduler_finished = 0;

void USAGE_STAT_START_OF_SCHEDULER_CALL(void){
	if(start_time_us != 0){
		scheduler_not_finished++;
		return;
	}
	scheduler_finished++;
	start_time_us = MICROS64();
}


#define avrg_1s_alpha 0.001f
void USAGE_STAT_END_OF_SCHEDULER_CALL(void){
	uint64_t now = MICROS64();
	uint32_t delta_t = now - start_time_us;
	avrg_1s = avrg_1s * (1.0f - avrg_1s_alpha) + delta_t * avrg_1s_alpha;
	if(delta_t > max_loop_time){
		max_loop_time = delta_t;
	}
	if(delta_t > max_loop_time_global){
		max_loop_time_global = delta_t;
	}
	if(now > second_reset_counter){
		second_reset_counter += 1000000;
		max_loop_time_1s = max_loop_time;
		max_loop_time = 0;
	}
	start_time_us = 0;
}

float USAGE_STAT_GET_AVRG_1S(void){
	return avrg_1s;
}

uint32_t USAGE_STAT_GET_MAX_LOOP_TIME_1S(void){
	return max_loop_time_1s;
}

float USAGE_STAT_GET_SCHEDULER_NOT_FINISHED_PERCENT(void){
	double total = (double)scheduler_finished + (double)scheduler_not_finished;
	if(total == 0){
		return 0;
	}
	return ((double)scheduler_not_finished / total) * 100.0;
}
