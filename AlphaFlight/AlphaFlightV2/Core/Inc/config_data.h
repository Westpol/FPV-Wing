/*
 * config_data.h
 *
 *  Created on: Oct 29, 2025
 *      Author: benno
 */

#ifndef INC_CONFIG_DATA_H_
#define INC_CONFIG_DATA_H_

#include <stdint.h>

typedef struct{
	uint8_t throttle;
	uint8_t pitch;
	uint8_t roll;
	uint8_t arm_switch;
	uint8_t mode_switch;
}config_data_crsf_channels_t;
typedef struct{
	uint8_t log_mode;
}config_data_sd_logger_setup_t;


extern config_data_crsf_channels_t CONFIG_DATA_CRSF_CHANNELS;
extern config_data_sd_logger_setup_t CONFIG_DATA_SD_LOGGER_SETUP;

#endif /* INC_CONFIG_DATA_H_ */
