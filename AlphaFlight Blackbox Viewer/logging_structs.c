
#include "logging_structs.h"
#include <string.h>
#include "main.h"

uint8_t copy_struct_onboard_sensors(const uint8_t* raw, uint8_t mode, FILE *file){
    LOG_ONBOARD_SENSORS_T log = {0};
    memcpy(&log, raw, sizeof(log));
    switch(mode){
        case 1:
            printf("%d,%d,%d,%f,%f,%f\n", log.header.log_type, log.header.log_version, log.header.timestamp, log.gyro_x_rad, log.gyro_y_rad, log.gyro_z_rad);
        break;

        case 2:
            fprintf(file, "%d,%d\n", log.header.log_type, log.header.timestamp);
        break;

        default:
        break;
    }
    return 1;
}

uint8_t copy_struct_crsf(const uint8_t* raw, uint8_t mode, FILE *file){
    LOG_CRSF_T log = {0};
    memcpy(&log, raw, sizeof(log));
    switch(mode){
        case 1:
            printf("%d,%d,%d,%d,%d,%d,%d\n", log.header.log_type, log.header.log_version, log.header.timestamp, log.channel_raw[0], log.channel_raw[1], log.channel_raw[2], log.channel_raw[3]);
        break;

        case 2:
            fprintf(file, "%d,%d\n", log.header.log_type, log.header.timestamp);
        break;

        default:
        break;
    }
    return 1;
}

uint8_t copy_struct_gps(const uint8_t* raw, uint8_t mode, FILE *file){
    LOG_GPS_T log = {0};
    memcpy(&log, raw, sizeof(log));
    switch(mode){
        case 1:
            printf("%d,%d,%d\n", log.header.log_type, log.header.log_version, log.header.timestamp);
        break;

        case 2:
            fprintf(file, "%d,%d\n", log.header.log_type, log.header.timestamp);
        break;

        default:
        break;
    }
    return 1;
}

uint8_t copy_struct_pid(const uint8_t* raw, uint8_t mode, FILE *file){
    LOG_PID_T log = {0};
    memcpy(&log, raw, sizeof(log));
    switch(mode){
        case 1:
            printf("%d,%d,%d\n", log.header.log_type, log.header.log_version, log.header.timestamp);
        break;

        case 2:
            fprintf(file, "%d,%d\n", log.header.log_type, log.header.timestamp);
        break;

        default:
        break;
    }
    return 1;
}