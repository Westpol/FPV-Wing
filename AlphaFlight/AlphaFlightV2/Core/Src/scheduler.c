/*
 * scheduler.c
 *
 *  Created on: Apr 7, 2025
 *      Author: benno
 */

#include "scheduler.h"
#include "time-utils.h"
#include "main.h"

static task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;
static uint32_t current_time = 0;



void SCHEDULER_INIT(){
	current_time = MICROS();
	for(int i = 0; i < task_count; i++){
		tasks[i].time_last_execute = current_time;
		tasks[i].time_to_execute = current_time + tasks[i].period;
	}
}

void SCHEDULER_ADD_TASK(task_func_t task_func, uint32_t period){	// add tasks in order of importance, tasks get checked/executed in the order that they were added
	if((task_count < MAX_TASKS) && period > MIN_TASK_DELAY){
		tasks[task_count].task_func = task_func;
		tasks[task_count].period = period;
		tasks[task_count].time_last_execute = current_time;
		tasks[task_count].time_to_execute = period;
		task_count++;
	}
	else{
		ERROR_HANDLER_BLINKS(1);
	}
}

void SCHEDULER_UPDATE(){
	current_time = MICROS();
	for(int i = 0; i < task_count; i++){
		if((int32_t)(current_time - tasks[i].time_to_execute) >= 0){
			tasks[i].time_last_execute = current_time;
			tasks[i].time_to_execute += tasks[i].period;
			tasks[i].task_func();
			break;
		}
	}
}
