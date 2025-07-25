/*
 * sd_logger.h
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#ifndef INC_SD_LOGGER_H_
#define INC_SD_LOGGER_H_

#include "stdint.h"
#include "stdbool.h"
#include "onboard-sensors.h"
#include "crossfire.h"
#include "m10-gps.h"

#define SUPERBLOCK_MAGIC 0xFA55C0DE
#define SUPERBLOCK_VERSION 1
#define SUPERBLOCK_BLOCK 99

#define LOG_VERSION 1


#define LOG_METADATA_BLOCK_MAGIC 0xC5D4250A

#define LOG_METADATA_MAGIC 0xA1F17E5C  // Unique marker for validity
#define LOG_METADATA_VERSION 1         // Version for forward compatibility
#define LOG_METADATA_BLOCK_START 100
#define LOG_METADATA_BLOCK_END 999
#define LOG_FILES_PER_METADATA_BLOCK 14

#define LOG_DATA_BLOCK_START 1000
#define LOG_DATA_BLOCK_END 59999999


#define MISSION_METADATA_BLOCK_START 60000000
#define MISSION_METADATA_BLOCK_END 60000099

#define MISSION_DATA_BLOCK_START 60000100
#define MISSION_DATA_BLOCK_END 61000000

#define BLOCK_SIZE     512
#define CRC32_BYTE_SIZE 4
#define TIMEOUT_MS     1000

typedef enum {
	ERROR_TIMEOUT = 1,
	ERROR_WRITE = 2,
	ERROR_CRC_MISMATCH = 3,
	ERROR_READ = 4,
	ERROR_BLOCK_LIMIT_REACHED = 5,
	ERROR_WRONG_MAGIC = 6,
	ERROR_DMA_WRITE = 7,
}ERROR_CODES;

typedef struct __attribute__((__packed__)) {
    uint32_t magic;           // Magic number to identify a valid struct
    uint16_t version;         // Metadata version
    uint8_t active_flag;	  // Flag to know the last active flight
    uint16_t flight_number;   // Monotonic flight counter

    uint32_t timestamp_unix;  // 0 if GPS time unavailable
    int32_t latitude;         // Scaled, e.g., degrees * 1e7
    int32_t longitude;        // Scaled, e.g., degrees * 1e7
    uint16_t gps_valid_flags; // Bit flags for GPS data validity

    uint32_t start_block;     // First SD block of the flight log
    uint32_t end_block;       // Last SD block, 0 = corrupted/incomplete

    uint8_t log_finished;     // 1 = log completed successfully

    uint8_t log_mode;		  // what data is logged e.g. Gyro, fly by wire, etc.
    uint8_t log_version;	  // which logger version

    uint8_t reserved[2];      // Padding / reserved for future use
} SD_FILE_METADATA_CHUNK;

typedef struct __attribute__((__packed__)){
	uint32_t magic;
	SD_FILE_METADATA_CHUNK sd_file_metadata_chunk[LOG_FILES_PER_METADATA_BLOCK];

}SD_FILE_METADATA_BLOCK;

typedef struct __attribute__((__packed__)) {
    uint32_t magic;               // Magic number to detect valid superblock
    uint16_t version;             // Version for compatibility

    // Log metadata and file block ranges
    uint32_t log_metadata_start_block;
    uint32_t log_metadata_end_block;
    uint32_t logfile_start_block;
    uint32_t logfile_end_block;

    // Mission metadata and file block ranges
    uint32_t mission_metadata_start_block;
    uint32_t mission_metadata_end_block;
    uint32_t missionfile_start_block;
    uint32_t missionfile_end_block;

    uint32_t card_size_MB;        // Size of the SD card in megabytes

    // Flight log statistics
    uint32_t relative_flight_num;
    uint32_t absolute_flight_num;

    uint8_t corruption_flag;      // 0 = OK, 1 = corrupted flight log detected

    uint32_t latest_mission_metadata_block;  // Last mission metadata block used

    // === NEW: Mission system and log config state ===
    uint32_t active_mission_id;       // ID of mission to run at startup (0 = none)
    uint8_t  mission_status_flags;    // Bitfield for current mission state
    uint8_t  mission_config_version;  // To version mission file format
    uint8_t  default_log_profile_id;  // Which logging config to use if none specified
    uint8_t  log_mode_flag;          // Flags for aggressive/debug/normal logging
} SD_SUPERBLOCK;

_Static_assert(sizeof(SD_SUPERBLOCK) <= 508, "Superblock struct too large for SD block!");
_Static_assert(sizeof(SD_FILE_METADATA_BLOCK) <= 508, "Metadata struct too large for SD block! Make Metadata chunk block smaller or reduce LOG_FILES_PER_METADATA_BLOCK");

uint32_t SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT);
void SD_LOGGER_LOOP_CALL();
void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS);

void SD_LOGGER_SETUP_CARD();

#endif /* INC_SD_LOGGER_H_ */
