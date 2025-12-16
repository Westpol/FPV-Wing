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
	uint64_t log_mode;		// each bit is one log topic, first bit = LOG_ENABLE
}config_data_internal_sd_logger_setup_t;

typedef struct{
	float p;
	float i;
	float i_limit;
	float i_zone;
	float d;
	float d_filter;
	float multiplier;
	float feed_forward;
}pid_value_helper_t;

typedef struct{
	pid_value_helper_t roll;
	pid_value_helper_t pitch;
}config_data_internal_pid_t;

typedef struct{
	float b[4];
	float k_i;
	float k_p;
}config_data_internal_mahony_values_t;

typedef struct{
	config_data_internal_crsf_t crossfire;
	config_data_internal_sd_logger_setup_t logger;
	config_data_internal_pid_t pid;
	config_data_internal_mahony_values_t mahony;
}config_data_t;

extern config_data_t CONFIG_DATA;

void CONFIG_DATA_BACKUP_DATA();
void CONFIG_DATA_COMPARE_TO_BACKUP();
uint8_t CONFIG_DATA_AVAILABLE();

#endif /* INC_CONFIG_DATA_H_ */
