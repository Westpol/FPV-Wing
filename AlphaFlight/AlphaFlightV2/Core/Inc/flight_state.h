/*
 * flight_state.h
 *
 *  Created on: Aug 19, 2025
 *      Author: benno
 */

#ifndef INC_FLIGHT_STATE_H_
#define INC_FLIGHT_STATE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t value;
} flight_magic_arm_t;

typedef struct {
    uint8_t value;
} flight_magic_rxloss_t;

#define FLIGHT_STATE_ARM_CHANGE_MAGIC 0xA5
#define FLIGHT_STATE_RXLOSS_CHANGE_MAGIC 0xF5

#define FLIGHT_STATE_ARM_CHANGE_KEY   ((flight_magic_arm_t){ .value = FLIGHT_STATE_ARM_CHANGE_MAGIC })
#define FLIGHT_STATE_RXLOSS_CHANGE_KEY   ((flight_magic_rxloss_t){ .value = FLIGHT_STATE_RXLOSS_CHANGE_MAGIC })

void FLIGHT_STATE_ARM(flight_magic_arm_t key);
void FLIGHT_STATE_DISARM(flight_magic_arm_t key);
bool FLIGHT_STATE_IS_ARMED();

void FLIGHT_STATE_RX_VALID(flight_magic_rxloss_t key);
void FLIGHT_STATE_RX_LOSS(flight_magic_rxloss_t key);
bool FLIGHT_STATE_IS_RX_LOSS();

#endif /* INC_FLIGHT_STATE_H_ */
