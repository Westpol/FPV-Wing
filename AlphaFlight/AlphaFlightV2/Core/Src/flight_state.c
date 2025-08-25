/*
 * flight_state.c
 *
 *  Created on: Aug 19, 2025
 *      Author: benno
 */

#include "flight_state.h"

#define FLIGHT_STATE_ARM_MAGIC 0x7C
#define FLIGHT_STATE_RXLOSS_MAGIC 0x65

static uint8_t flight_flag = 0;
static const uint8_t *flight_state_messages[3] = {(const uint8_t*)"disarm", (const uint8_t*)"arm", (const uint8_t*)"armfail"};

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

void FLIGHT_STATE_ARM(flight_magic_arm_t KEY){
	if(KEY.value != FLIGHT_STATE_ARM_CHANGE_MAGIC) return;
	flight_state.armed = FLIGHT_STATE_ARM_MAGIC;
}

void FLIGHT_STATE_DISARM(flight_magic_arm_t KEY){
	(void)KEY;
	flight_state.armed = 0;
}

bool FLIGHT_STATE_IS_ARMED(void){
	if(flight_state.armed == FLIGHT_STATE_ARM_MAGIC) return 1;
	if(flight_state.armed == 0) return 0;
	RECOVER_ARM_STATUS();
	if(flight_state.armed == FLIGHT_STATE_ARM_MAGIC) return 1;
	if(flight_state.armed == 0) return 0;
	return 0;
}

void FLIGHT_STATE_RX_VALID(flight_magic_rxloss_t KEY){
	if(KEY.value != FLIGHT_STATE_RXLOSS_CHANGE_MAGIC) return;
		flight_state.rxloss = 0;
}

void FLIGHT_STATE_RX_LOSS(flight_magic_rxloss_t KEY){
	(void)KEY;
	flight_state.rxloss = FLIGHT_STATE_RXLOSS_MAGIC;
}

bool FLIGHT_STATE_IS_RX_LOSS(void){
	if(flight_state.rxloss == 0) return 0;
	if(flight_state.rxloss == FLIGHT_STATE_RXLOSS_MAGIC) return 1;
	flight_state.rxloss = FLIGHT_STATE_RXLOSS_MAGIC;
	return 1;
}

void FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_ARMING FLIGHT_STATE){
	flight_flag = FLIGHT_STATE;
}

uint8_t FLIGHT_STATE_GET_FLAG(void){
	return flight_flag;
}

const uint8_t* FLIGHT_STATE_GET_STATE_STRING(void){
	return flight_state_messages[flight_flag];
}
