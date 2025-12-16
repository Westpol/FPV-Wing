#include "log_reader.h"
#include "logging_structs.h"
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "main.h"

uint8_t super_block_buffer[512] = {0};
uint8_t metadata_block_buffer[512] = {0};
int fd;
SD_SUPERBLOCK sd_superblock;
SD_FILE_METADATA_BLOCK sd_metadata_block;
SD_FILE_METADATA_CHUNK flight_metadada;

DECODER_T decoder[] = {{LOG_TYPE_DISABLE_LOGGING, NULL}, {LOG_TYPE_ONBOARD_SENSORS, copy_struct_onboard_sensors}, {LOG_TYPE_CRSF, copy_struct_crsf}, {LOG_TYPE_GPS, copy_struct_gps}, {LOG_TYPE_PID, copy_struct_pid}};


#define START_MAGIC ((uint16_t)0xC85A)
#define END_MAGIC ~((uint16_t)0xC85A)

uint32_t crc32_stm32(const uint8_t* data, int length)
{
    uint32_t crc = 0xFFFFFFFF;

    size_t i = 0;
    while (length >= 4) {
        uint32_t word = data[i] | (data[i+1] << 8) | (data[i+2] << 16) | (data[i+3] << 24);
        // simulate __REV(word)
        word = ((word & 0xFF) << 24) | ((word & 0xFF00) << 8) | ((word & 0xFF0000) >> 8) | ((word & 0xFF000000) >> 24);

        crc ^= word;

        for (int b = 0; b < 32; b++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }

        i += 4;
        length -= 4;
    }

    if (length > 0) {
        uint32_t word = 0;
        for (int j = 0; j < length; j++)
            word |= data[i+j] << (8 * j); // little-endian into lower bytes

        // simulate __REV(word)
        word = ((word & 0xFF) << 24) | ((word & 0xFF00) << 8) | ((word & 0xFF0000) >> 8) | ((word & 0xFF000000) >> 24);

        crc ^= word;

        for (int b = 0; b < 32; b++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }

    return crc; // this is exactly what hardware would output in DR
}



static uint32_t FLIGHT_NUM_TO_BLOCK(uint32_t relative_flight_num){
	uint32_t block = 0;

	block = LOG_METADATA_BLOCK_START + relative_flight_num / LOG_FILES_PER_METADATA_BLOCK;

	return block;
}

static uint8_t FLIGHT_NUM_TO_INDEX(uint32_t relative_flight_num){
	uint8_t index = 0;

	index = relative_flight_num % LOG_FILES_PER_METADATA_BLOCK;

	return index;
}

static int READ_SINGLE_BLOCK(uint8_t* BUFFER, uint32_t BLOCK){
    off_t offset = BLOCK * BLOCK_SIZE;
    if (lseek(fd, offset, SEEK_SET) != offset) {
        perror("Seek failed");
        return 1;
    }

    ssize_t bytes_read = read(fd, BUFFER, BLOCK_SIZE);
    if (bytes_read != BLOCK_SIZE) {
        perror("Read failed");
        return 1;
    }

    #if ENABLE_CRC
    uint32_t calculated_block_crc32 = crc32_stm32(BUFFER, 508);
    uint32_t block_crc;
    memcpy(&block_crc, BUFFER + 508, sizeof(block_crc));
    if(block_crc != calculated_block_crc32){
        printf("Problem with Block %d\n", BLOCK);
        printf("Calculated CRC: %08X\n", calculated_block_crc32);
        printf("Block CRC     : %08X\n", block_crc);
        //fprintf(stderr, "Block CRC wrong");
        return 1;
    }
    #endif

    return 0;
}

static void PRINT_FLIGHT_DATA(){
    uint8_t block_buffer[512];
    for(uint32_t i = flight_metadada.start_block; i <= flight_metadada.end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        uint16_t start_magic = 0;
        uint16_t end_magic = 0;
        for(int i = 0; i < 504; i++){

            start_magic = 0;
            end_magic = 0;

            start_magic = ((uint16_t)block_buffer[i+1] << 8) | (uint16_t)block_buffer[i];

            if(start_magic == START_MAGIC){
                printf("STRUCT FOUND AT INDEX %d\n", i);
                //int struct_length = block_buffer[i + 4];
                //printf("STRUCT LENGTH: %d\n", struct_length);
                int struct_id = block_buffer[i + 3];
                //printf("STRUCT ID: %d\n", struct_id);

                end_magic = ((uint16_t)block_buffer[i + block_buffer[i + 2] - 2 + 1] << 8) | ((uint16_t)block_buffer[i + block_buffer[i + 2] - 2]);
                printf("END MAGIC: 0x%08X\n\n", end_magic);

                decoder[struct_id].decode(&block_buffer[i], 1, NULL);


                i += block_buffer[i+2] - 1;

                continue;
            }
        }
    }
}

static void EXPORT_FLIGHT(FILE *file){
    uint8_t block_buffer[512];
    for(uint32_t i = flight_metadada.start_block; i <= flight_metadada.end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        uint16_t start_magic = 0;
        uint16_t end_magic = 0;
        for(int i = 0; i < 504; i++){

            start_magic = 0;
            end_magic = 0;

            start_magic = ((uint16_t)block_buffer[i+1] << 8) | (uint16_t)block_buffer[i];

            if(start_magic == START_MAGIC){
                printf("STRUCT FOUND AT INDEX %d\n", i);
                //int struct_length = block_buffer[i + 4];
                //printf("STRUCT LENGTH: %d\n", struct_length);
                int struct_id = block_buffer[i + 3];
                //printf("STRUCT ID: %d\n", struct_id);

                end_magic = ((uint16_t)block_buffer[i + block_buffer[i + 2] - 2 + 1] << 8) | ((uint16_t)block_buffer[i + block_buffer[i + 2] - 2]);
                printf("END MAGIC: 0x%08X\n\n", end_magic);

                decoder[struct_id].decode(&block_buffer[i], 2, file);


                i += block_buffer[i+2] - 1;

                continue;
            }
        }
    }
}

static void DUMP_FLIGHT_TO_BIN(const char* filename){
    const int start_block = flight_metadada.start_block;
    const int end_block = flight_metadada.end_block;
    uint8_t block_buffer[512] = {0};
    FILE* of = fopen(filename, "wb");
    for(int i = start_block; i <= end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        fwrite(block_buffer, 1, BLOCK_SIZE, of);
    }
    fclose(of);
}

int INITIALIZE_SD_CARD(const char* PATH, bool ENABLE_BIN_FILE, const char* BIN_FILENAME, bool ENABLE_CSV_FILE, const char* CSV_FILENAME, bool PRINT_ENABLE){
    fd = open(PATH, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    if (READ_SINGLE_BLOCK(super_block_buffer, SUPERBLOCK_INDEX) != 0) {
        fprintf(stderr, "Failed to read superblock\n");
        //close(fd);
        return 1;
    }
    memcpy(&sd_superblock, &super_block_buffer, sizeof(sd_superblock));
    
    if(sd_superblock.magic != SUPERBLOCK_MAGIC){
        printf("Wrong superblock magic number\n");
        return 1;
    }
    int minimum_flight_num = sd_superblock.absolute_flight_num - sd_superblock.relative_flight_num;
    int maximum_flight_num = sd_superblock.relative_flight_num - 1;
    if(sd_superblock.relative_flight_num == 0){
        printf("No flights on SD card!\n");
        return 1;
    }
    printf("Choose between flight number %d and %d: ", minimum_flight_num, maximum_flight_num);
    int flight_chosen;
    while(1){
        scanf("%d", &flight_chosen);
        if(flight_chosen >= minimum_flight_num && flight_chosen <= maximum_flight_num) break;
        printf("Wrong input. Try again.\n");
    }

    if (READ_SINGLE_BLOCK(metadata_block_buffer, FLIGHT_NUM_TO_BLOCK(flight_chosen)) != 0) {
        fprintf(stderr, "Failed to read metadata block\n");
        //close(fd);
        return 1;
    }
    memcpy(&sd_metadata_block, &metadata_block_buffer, sizeof(sd_metadata_block));
    flight_metadada = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(flight_chosen)];




    #if VERBOSE_OUTPUT
    printf("Superblock version: %d\nRelative flight number: %d\nMagic number: %08X\n", sd_superblock.version, sd_superblock.relative_flight_num, sd_superblock.magic);
    for(int i = 0; i < sd_superblock.relative_flight_num; i++){
        //printf("Flight %d, read block %d, index %d\n", i, FLIGHT_NUM_TO_BLOCK(i), FLIGHT_NUM_TO_INDEX(i));
        printf("Flight %d, active flag %d, completion flag %d, start block %d, end block %d\n", sd_metadata_block.sd_file_metadata_chunk[i].flight_number, sd_metadata_block.sd_file_metadata_chunk[i].active_flag, sd_metadata_block.sd_file_metadata_chunk[i].log_finished, sd_metadata_block.sd_file_metadata_chunk[i].start_block, sd_metadata_block.sd_file_metadata_chunk[i].end_block);
    }
    #endif




    printf("Flight %d, active flag %d, completion flag %d, start block %d, end block %d\n\n", flight_metadada.flight_number, flight_metadada.active_flag, flight_metadada.log_finished, flight_metadada.start_block, flight_metadada.end_block);
    printf("Active log types:\n");

    const char* log_types[64];
    log_types[1] = "ONBOARD_SENSORS";
    log_types[2] = "CRSF";
    log_types[3] = "GPS";
    log_types[4] = "PID";
    for(int i = 1; i < 64; i++){
        if((flight_metadada.log_mode >> i) && 1){
            printf("%s\n", log_types[i]);
        }
    }
    printf("\n");

    char buff[21];
    const long int time_unix = flight_metadada.timestamp_unix;
    strftime(buff, 20, "%d.%m.%Y %H:%M:%S", localtime(&time_unix));
    buff[20] = '\0';
    printf("%s\n\n", buff);

    if(PRINT_ENABLE){
        printf("Printing Data...\n");
        PRINT_FLIGHT_DATA();
        printf("\n\nDone.");
    }

    if(ENABLE_BIN_FILE){
        printf("\n\nDumping data to %s...\n", BIN_FILENAME);
        DUMP_FLIGHT_TO_BIN(BIN_FILENAME);
        printf("Done.\n");
    }

    if(ENABLE_CSV_FILE){
        printf("\nExporting to %s...\n", CSV_FILENAME);
        FILE *csv_file = fopen(CSV_FILENAME, "w");
        fprintf(csv_file, "HEADER\n");
        EXPORT_FLIGHT(csv_file);
        fclose(csv_file);
        printf("Done.\n");
    }

    close(fd);
    return 0;
}
