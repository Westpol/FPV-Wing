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

#define METADATA_BLOCK_MAGIC 0xC5D4250A

#define METADATA_MAGIC 0xA1F17E5C  // Unique marker for validity
#define METADATA_VERSION 1         // Version for forward compatibility
#define METADATA_BLOCK_START 101
#define METADATA_BLOCK_END 999
#define FILES_PER_METADATA_BLOCK 14

#define SUPERBLOCK_MAGIC 0xFA55C0DE
#define SUPERBLOCK_VERSION 1
#define SUPERBLOCK_BLOCK 100

#define DATA_BLOCK_START 1000

#define BLOCK_SIZE     512
#define TIMEOUT_MS     1000

typedef struct __attribute__((__packed__, aligned(4))) {
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
    uint8_t reserved[4];      // Padding / reserved for future use
} SD_FILE_METADATA_CHUNK;

typedef struct __attribute__((__packed__, aligned(4))){
	uint32_t magic;
	SD_FILE_METADATA_CHUNK sd_file_metadata_chunk[FILES_PER_METADATA_BLOCK];

	uint32_t crc32;
}SD_FILE_METADATA_BLOCK;

typedef struct __attribute__((__packed__, aligned(4))) {
    uint32_t magic;               // Magic number to detect valid superblock
    uint16_t version;             // Version for compatibility

    uint32_t file_start_block;        // Total number of 512-byte blocks on the card
    uint32_t file_end_block;        // Total number of 512-byte blocks on the card
    uint32_t card_size_MB;        // Size of the SD card in megabytes

    uint32_t total_flights;       // Total number of flights ever logged
    uint32_t last_flight_number;  // Last flight number used

    uint8_t corruption_flag;      // 0 = OK, 1 = corrupted flight log detected
    uint8_t reserved1[3];         // Align + room for future flags

    uint32_t latest_metadata_block;  // Which block holds the latest flight metadata

    uint8_t reserved2[472];       // Padding to make struct 508 bytes total

    uint32_t crc32;               // CRC32 of everything except this field
} SD_SUPERBLOCK;

void SD_LOGGER_INIT(Sensor_Data* SENSOR_DATA, CRSF_DATA* CRSF_DATA, GPS_NAV_PVT* GPS_NAV_PVT);
void SD_LOGGER_LOOP_CALL();
void SD_LOGGER_FORWARD_ARM(bool ARM_STATUS);

void SD_LOGGER_SETUP_CARD();

#endif /* INC_SD_LOGGER_H_ */
