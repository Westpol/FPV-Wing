/*
 * flight_control.c
 *
 *  Created on: Apr 17, 2025
 *      Author: benno
 */

#include "flight_control.h"
#include "time-utils.h"
#include "utils.h"
#include "stdbool.h"
#include "attitude_pid.h"
#include "debug.h"
#include "sd_logger.h"
#include "m10-gps.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "flight_state.h"
#include "config_data.h"

extern IMU_Data imu_data;
extern GPS_NAV_PVT gps_nav_pvt;
extern CRSF_DATA crsf_data;
static FLIGHT_MODE current_flight_mode = DIRECT_CONTROL;
FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints = {0};
static bool arm_failed = false;

static uint64_t last_process_execution_time = 0;

void FC_SANITY_CHECK(){
	if(crsf_data.last_channel_update + 5000000 < MICROS64()){
		FLIGHT_STATE_RX_LOSS(FLIGHT_STATE_RXLOSS_CHANGE_KEY);
	}
	else if(FLIGHT_STATE_IS_RX_LOSS()){
		FLIGHT_STATE_RX_VALID(FLIGHT_STATE_RXLOSS_CHANGE_KEY);
	}
	/*if(gps_nav_pvt.numSV < 5){	TODO: Implement rxloss fallback logic
		if(current_flight_mode == AUTOPILOT && !rx_lost){
			current_flight_mode = FLY_BY_WIRE;
		}
		else if(rx_lost){
			current_flight_mode = RETURN_TO_HOME_NO_GPS;
		}
	}*/
}

void FC_MODE_CHECK(){
	if(!FLIGHT_STATE_IS_RX_LOSS()){
		if(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.arm_switch] > 60 && FLIGHT_STATE_IS_ARMED() == false && arm_failed == false){
			if(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.throttle] < 1){
				FLIGHT_STATE_ARM(FLIGHT_STATE_ARM_CHANGE_KEY);
				FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_ARMED);
			}
			else{
				arm_failed = true;
				FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_ARM_FAILED);
			}
		}
		if((crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.arm_switch] < 60 && FLIGHT_STATE_IS_ARMED()) || (crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.arm_switch] < 60 && arm_failed == true)){
			FLIGHT_STATE_DISARM(FLIGHT_STATE_ARM_CHANGE_KEY);
			arm_failed = false;
			FLIGHT_STATE_SET_EVENT(FLIGHT_STATE_DISARMED);
		}

		if(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.mode_switch] < 10){
			current_flight_mode = DIRECT_CONTROL;
			fly_by_wire_setpoints.roll_angle = imu_data.angle_x_fused;
			fly_by_wire_setpoints.pitch_angle = imu_data.angle_y_fused;
		}
		else if(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.mode_switch] < 60){
			current_flight_mode = FLY_BY_WIRE;
		}
		else{
			//current_flight_mode = AUTOPILOT;
			current_flight_mode = FLY_BY_WIRE;
		}
	}
}

static float FC_CRSF_DEADBAND(float value, float deadband_width, float deadband_point){
	if(value > (deadband_point - deadband_width) && value < (deadband_point + deadband_width)){
		return deadband_point;
	}
	return value;
}

void FC_PROCESS(){
	uint64_t dt = MICROS64() - last_process_execution_time;
	last_process_execution_time = MICROS64();
	if(dt > 100000){
		return;
	}

	fly_by_wire_setpoints.pitch_angle = UTIL_MIN_F(UTIL_MAX_F(fly_by_wire_setpoints.pitch_angle - (((FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.pitch], 0.5, 50) / 50.0f - 1) * 10.0f) / (1000000.0f / dt)), 30), -25);
	fly_by_wire_setpoints.roll_angle = UTIL_MIN_F(UTIL_MAX_F(fly_by_wire_setpoints.roll_angle - (((FC_CRSF_DEADBAND(crsf_data.channel_norm[CONFIG_DATA_CRSF_CHANNELS.roll], 0.5, 50) / 50.0f - 1) * 15.0) / (1000000.0f / dt)), 45), -45);

	switch (current_flight_mode) {
		case DIRECT_CONTROL:
			FC_PID_DIRECT_CONTROL();
			break;
		case DIRECT_CONTROL_WITH_LIMITS:
			break;
		case FLY_BY_WIRE:
			FC_PID_FLY_BY_WIRE_WITHOUT_LIMITS(dt);
			break;
		default:
			break;
	}
}
