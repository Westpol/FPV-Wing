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
}config_data_internal_crsf_channels_t;

typedef struct{
	config_data_internal_crsf_channels_t channels;
}config_data_internal_crsf_t;

typedef struct{
	uint8_t log_mode;
}config_data_internal_sd_logger_setup_t;

typedef struct{
	config_data_internal_crsf_t crossfire;
	config_data_internal_sd_logger_setup_t logger;
}config_data_t;

extern config_data_t CONFIG_DATA;

#endif /* INC_CONFIG_DATA_H_ */
