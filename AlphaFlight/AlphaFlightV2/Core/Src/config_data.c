/*
 * config_data.c
 *
 *  Created on: Oct 29, 2025
 *      Author: benno
 */

#include "config_data.h"
#include <string.h>

#include "main.h"
#include "flight_state.h"
#include "load_config.h"

config_data_t CONFIG_DATA = {0};
config_data_t config_data_backup = {0};

void CONFIG_DATA_BACKUP_DATA(){
	memcpy(&config_data_backup, &CONFIG_DATA, sizeof(config_data_t));
	if(memcmp(&config_data_backup, &CONFIG_DATA, sizeof(config_data_t)) != 0){
	#ifdef DEBUG_ENABLED
		ERROR_HANDLER_BLINKS(10);
	#endif
	}
}

void CONFIG_DATA_COMPARE_TO_BACKUP(){
	if(FLIGHT_STATE_IS_USB_CONNECTED() == 0){
		if(memcmp(&config_data_backup, &CONFIG_DATA, sizeof(config_data_t)) != 0){
		#ifdef DEBUG_ENABLED
			ERROR_HANDLER_BLINKS(10);
		#endif
			memcpy(&CONFIG_DATA, &config_data_backup, sizeof(config_data_t));
		}
	}
}

uint8_t CONFIG_DATA_AVAILABLE(){
	return CONFIG_WAS_READ();
}
