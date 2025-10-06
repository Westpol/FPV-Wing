/*
 * usage_stats.h
 *
 *  Created on: Oct 4, 2025
 *      Author: benno
 */

#ifndef INC_USAGE_STATS_H_
#define INC_USAGE_STATS_H_

#include <stdint.h>

void USAGE_STAT_START_OF_SCHEDULER_CALL(void);
void USAGE_STAT_END_OF_SCHEDULER_CALL(void);

float USAGE_STAT_GET_AVRG_1S(void);
uint32_t USAGE_STAT_GET_MAX_LOOP_TIME_1S(void);
float USAGE_STAT_GET_SCHEDULER_NOT_FINISHED_PERCENT(void);

#endif /* INC_USAGE_STATS_H_ */
