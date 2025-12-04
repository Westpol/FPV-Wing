
#include "logging_structs.h"
#include <string.h>
#include <stdio.h>

uint8_t copy_struct_onboard_sensors(const uint8_t* raw, void* data){
    memcpy(data, raw, sizeof(LOG_ONBOARD_SENSORS_T));
}

void print_struct_onboard_sensors(void* data){
    LOG_ONBOARD_SENSORS_T* log = data;
    printf("%d,%d,%ld,%f,%f,%f,", log->header.log_type, log->header.log_version, log->header.timestamp, log->gyro_x_rad, log->gyro_y_rad, log->gyro_z_rad);
}