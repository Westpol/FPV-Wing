#ifndef LOG_READER_H
#define LOG_READER_H

#include "stdint.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdbool.h"

#define BLOCK_SIZE 512
#define SUPERBLOCK_INDEX 99
#define LOG_METADATA_BLOCK_START 100
#define LOG_FILES_PER_METADATA_BLOCK 14
#define ENABLE_CRC false
#define VERBOSE_OUTPUT false

#define SUPERBLOCK_MAGIC 0xFA55C0DE
#define LOG_METADATA_BLOCK_MAGIC 0xC5D4250A
#define LOG_METADATA_MAGIC 0xA1F17E5C

#define LOG_FRAME_START_MAGIC 0xC8
#define LOG_FRAME_END_MAGIC 0x9A

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

typedef struct __attribute__((packed)){
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

typedef struct __attribute__((packed)){
	uint16_t start_magic;

	uint64_t timestamp;

	float angle_fused_x, angle_fused_y, angle_fused_z;		// onboard-sensors

	float baro_altimeter;

	double gps_lon, gps_lat, gps_height, gps_speed, gps_heading;		// GPS
	uint8_t gps_sats;

	uint16_t crsf_ch[4];		// user inputs

	uint16_t status_flags;

	float pid_correction_roll, pid_correction_pitch;

	uint16_t end_magic;

}T1V0_GENERAL_DATA;

int INITIALIZE_SD_CARD(const char* PATH);

#endif