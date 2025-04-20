/*
 * flight_control.c
 *
 *  Created on: Apr 17, 2025
 *      Author: benno
 */

#include "flight_control.h"
#include "time-utils.h"
#include "stdbool.h"
#include "attitude_pid.h"
#include "debug.h"

static Sensor_Data *sensor_data;
static GPS_NAV_PVT *gps_nav_pvt;
static CRSF_DATA *crsf_data;
static FLIGHT_MODE current_flight_mode = DIRECT_CONTROL;
static FLY_BY_WIRE_SETPOINTS fly_by_wire_setpoints = {0};
static bool rx_lost = false;


void FC_INIT(Sensor_Data *sensor_d, GPS_NAV_PVT *gps_nav, CRSF_DATA *crsf_d){
	sensor_data = sensor_d;
	gps_nav_pvt = gps_nav;
	crsf_data = crsf_d;

}

void FC_SANITY_CHECK(){
	if(crsf_data->last_channel_update + 1000000 < MICROS()){
		rx_lost = true;
	}
	else if(rx_lost){
		rx_lost = false;
	}
	if(gps_nav_pvt->numSV < 5){
		if(current_flight_mode == AUTOPILOT && !rx_lost){
			current_flight_mode = FLY_BY_WIRE;
		}
		else if(rx_lost){
			current_flight_mode = RETURN_TO_HOME_NO_GPS;
		}
	}
}

void FC_MODE_CHECK(){
	if(!rx_lost){
		STATUS_LED_GREEN_ON();
		if(crsf_data->channel[5] < 500){
			current_flight_mode = AUTOPILOT;
		}
		else if(crsf_data->channel[5] < 1500){
			current_flight_mode = FLY_BY_WIRE;
		}
		else{
			current_flight_mode = DIRECT_CONTROL;
		}
		STATUS_LED_GREEN_OFF();
	}
}

void FC_PROCESS(){
	switch (current_flight_mode) {
		case DIRECT_CONTROL:
			FC_PID_DIRECT_CONTROL();
			break;
		case DIRECT_CONTROL_WITH_LIMITS:
			break;
		default:
			break;
	}
}

FLY_BY_WIRE_SETPOINTS *FC_GET_SETPOINTS(){
	return &fly_by_wire_setpoints;
}
