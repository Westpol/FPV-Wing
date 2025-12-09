
#include "logging_structs.h"
#include <string.h>
#include <stdio.h>

uint8_t copy_struct_onboard_sensors(const uint8_t* raw, void* data, uint8_t mode){
    memcpy(data, raw, sizeof(LOG_ONBOARD_SENSORS_T));
    return 1;
}

void print_struct_onboard_sensors(const void* data){
    const LOG_ONBOARD_SENSORS_T* log = data;
    printf("%d,%d,%ld,%f,%f,%f\n", log->header.log_type, log->header.log_version, log->header.timestamp, log->gyro_x_rad, log->gyro_y_rad, log->gyro_z_rad);
}

uint8_t copy_struct_crsf(const uint8_t* raw, void* data, uint8_t mode){
    memcpy(data, raw, sizeof(LOG_CRSF_T));
    return 1;
}

void print_struct_crsf(const void* data){
    const LOG_CRSF_T* log = data;
    printf("%d,%d,%ld,%d,%d,%d,%d\n", log->header.log_type, log->header.log_version, log->header.timestamp, log->channel_raw[0], log->channel_raw[1], log->channel_raw[2], log->channel_raw[3]);
}