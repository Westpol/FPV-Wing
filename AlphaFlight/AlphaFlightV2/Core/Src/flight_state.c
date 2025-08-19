/*
 * flight_state.c
 *
 *  Created on: Aug 19, 2025
 *      Author: benno
 */

#include "flight_state.h"

#define FLIGHT_STATE_ARM_MAGIC 0x7C
#define FLIGHT_STATE_RXLOSS_MAGIC 0x65

static struct {
    uint8_t armed;
    uint8_t rxloss;
} flight_state = {.armed = 0, .rxloss = FLIGHT_STATE_RXLOSS_MAGIC};

static void RECOVER_ARM_STATUS(){
	if(flight_state.armed == 0 || flight_state.armed == FLIGHT_STATE_ARM_MAGIC){
		return;
	}
	flight_state.armed = 0;
}

void FLIGHT_STATE_ARM(flight_magic_arm_t key){
	if(key.value != FLIGHT_STATE_ARM_CHANGE_MAGIC) return;
	flight_state.armed = FLIGHT_STATE_ARM_MAGIC;
}

void FLIGHT_STATE_DISARM(flight_magic_arm_t key){
	(void)key;
	flight_state.armed = 0;
}

bool FLIGHT_STATE_IS_ARMED(){
	if(flight_state.armed == FLIGHT_STATE_ARM_MAGIC) return 1;
	if(flight_state.armed == 0) return 0;
	RECOVER_ARM_STATUS();
	if(flight_state.armed == FLIGHT_STATE_ARM_MAGIC) return 1;
	if(flight_state.armed == 0) return 0;
	return 0;
}

void FLIGHT_STATE_RX_VALID(flight_magic_rxloss_t key){
	if(key.value != FLIGHT_STATE_RXLOSS_CHANGE_MAGIC) return;
		flight_state.rxloss = 0;
}

void FLIGHT_STATE_RX_LOSS(flight_magic_rxloss_t key){
	(void)key;
	flight_state.rxloss = FLIGHT_STATE_RXLOSS_MAGIC;
}

bool FLIGHT_STATE_IS_RX_LOSS(){
	if(flight_state.rxloss == 0) return 0;
	if(flight_state.rxloss == FLIGHT_STATE_RXLOSS_MAGIC) return 1;
	flight_state.rxloss = FLIGHT_STATE_RXLOSS_MAGIC;
	return 1;
}
