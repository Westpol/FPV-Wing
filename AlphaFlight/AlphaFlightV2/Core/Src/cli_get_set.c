/*
 * cli_get_set.c
 *
 *  Created on: Dec 17, 2025
 *      Author: benno
 */

#include "cli_get_set.h"
#include <string.h>
#include "main.h"
#include "debug.h"
#include "config_data.h"

uint8_t CLI_PROCESS_GET_COMMAND(const char* get_type){
	uint8_t len_commands = 5;
	const char* commands[] = {"pid", "crsf", "gyro", "filter", "logger"};
	for(int i = 0; i < len_commands; i++){
		if(strcmp(get_type, commands[i]) != 0) continue;

		switch(i){
		case 0:
			USB_PRINTLN("pid roll i");
			break;
		case 1:
			USB_PRINTLN("crsf telemetry %d", CONFIG_DATA.crossfire.telemetry.enabled);
			break;
		case 2:
			USB_PRINTLN("gyro roll");
			break;
		case 3:
			USB_PRINTLN("filter roll");
			break;
		case 4:
			USB_PRINTLN("logger mode %d", CONFIG_DATA.logger.log_mode);
			break;
		default:
			ERROR_HANDLER_BLINKS(5);
		}
		return 1;
	}
	return 0;
}
