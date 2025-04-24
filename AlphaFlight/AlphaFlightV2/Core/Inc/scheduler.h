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

#define MAX_TASKS 20

typedef void (*task_func_t)(void);

typedef struct{
	task_func_t task_func;
	uint32_t period;
	uint32_t time_last_execute;
	uint32_t time_to_execute;
}task_t;

void SCHEDULER_INIT();
void SCHEDULER_ADD_TASK(task_func_t task_func, uint32_t period);
void SCHEDULER_UPDATE(void);

#endif /* INC_SCHEDULER_H_ */
