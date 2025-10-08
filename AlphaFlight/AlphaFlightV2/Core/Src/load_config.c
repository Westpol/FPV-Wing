/*
 * load_config.c
 *
 *  Created on: Oct 8, 2025
 *      Author: benno
 */

#include "load_config.h"

#include <stdint.h>

#include "sd.h"

#include "attitude_pid.h"			// includes to so that I can extern structs
#include "crossfire.h"
#include "flight_control.h"
#include "onboard-sensors.h"
#include "m10-gps.h"
#include "sd_logger.h"
#include "debug.h"

void print_block_blocking(uint32_t block){
	uint8_t bytes[512];

	SD_READ_BLOCK(bytes, block);
	for(int i = 0; i < 32; i++){
		USB_PRINTLN_BLOCKING("%02X %02X %02X %02X %02X %02X %02X %02X    %02X %02X %02X %02X %02X %02X %02X %02X", bytes[0+i*16], bytes[1+i*16], bytes[2+i*16], bytes[3+i*16], bytes[4+i*16], bytes[5+i*16], bytes[6+i*16], bytes[7+i*16], bytes[8+i*16], bytes[9+i*16], bytes[10+i*16], bytes[11+i*16], bytes[12+i*16], bytes[13+i*16], bytes[14+i*16], bytes[15+i*16]);
	}
}
