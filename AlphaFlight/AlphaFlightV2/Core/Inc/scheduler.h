/*
 * scheduler.h
 *
 *  Created on: Apr 7, 2025
 *      Author: benno
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "stm32f7xx_hal.h"
#include "stdint.h"

#define MAX_TASKS 50
#define MIN_TASK_DELAY 1000		// in microseconds (default 1000: equals 1ms or 1kHz)

typedef void (*task_func_t)(void);

typedef struct{
	task_func_t task_func;
	uint32_t period;
	uint64_t time_last_execute;
	uint64_t time_to_execute;
	float cpu_usage;
	char name[32];
}task_t;

void SCHEDULER_INIT();
void SCHEDULER_CHECK_EXECUTION_DELAY();
void SCHEDULER_ADD_TASK(const task_func_t task_func, const uint32_t period, const char* name);
void SCHEDULER_UPDATE(void);

#define HZ_TO_DELTA_T_US(HZ) ((uint32_t)(1000000U / (HZ)))

#endif /* INC_SCHEDULER_H_ */
