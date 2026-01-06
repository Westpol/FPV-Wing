/*
 * dshot.h
 *
 *  Created on: Jan 6, 2026
 *      Author: benno
 */

#ifndef INC_DSHOT_H_
#define INC_DSHOT_H_

#include <stdint.h>

#define DSHOT600_COUNTER_PERIOD_TICKS 360
#define DSHOT600_COUNTER_HIGH_PULSE_TICKS 270
#define DSHOT600_COUNTER_LOW_PULSE_TICKS 135

void DSHOT_DMA_TRANSFER_COMPLETED(void);
void DSHOT_TIMER_INTERRUPT(void);
void DSHOT_SEND_FRAME(void);
void DSHOT_SET_THROTTLE(uint8_t channel, uint16_t value);

#endif /* INC_DSHOT_H_ */
