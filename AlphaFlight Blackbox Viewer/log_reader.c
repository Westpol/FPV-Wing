    #include "log_reader.h"
    #include "stdio.h"
    #include "stdint.h"
    #include <fcntl.h>
    #include <unistd.h>
    #include <string.h>
    #include "stdbool.h"

    uint8_t super_block_buffer[512] = {0};
    uint8_t metadata_block_buffer[512] = {0};
    int fd;
    SD_SUPERBLOCK sd_superblock;
    SD_FILE_METADATA_BLOCK sd_metadata_block;

uint32_t crc32_stm32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;

    size_t i = 0;
    while (i + 4 <= length) {
        // LSB-first loading (little-endian word)
        uint32_t word = ((uint32_t)data[i]) |
                        ((uint32_t)data[i + 1] << 8) |
                        ((uint32_t)data[i + 2] << 16) |
                        ((uint32_t)data[i + 3] << 24);
        crc ^= word;

        for (int b = 0; b < 32; b++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }

        i += 4;
    }

    // Handle remaining bytes (pad with zeroes)
    if (i < length) {
        uint32_t word = 0;
        for (int j = 0; i < length; ++i, ++j) {
            word |= ((uint32_t)data[i]) << (j * 8);  // LSB-first
        }
        crc ^= word;
        for (int b = 0; b < 32; b++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ 0x04C11DB7;
            else
                crc <<= 1;
        }
    }

    return crc;
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

static void DUMP_FLIGHT_TO_BIN(int chosen_flight){
    const int start_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].start_block;
    const int end_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].end_block;
    uint8_t block_buffer[512] = {0};
    FILE* of = fopen("test.bin", "wb");
    for(int i = start_block; i <= end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        fwrite(block_buffer, 1, BLOCK_SIZE, of);
    }
    fclose(of);
}

static void PRINT_FLIGHT_DATA(int chosen_flight){
    const int start_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].start_block;
    const int end_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].end_block;
    uint8_t block_buffer[512] = {0};
    for(int i = start_block; i <= end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        int block_position = 0;
        int entry_length = sizeof(LOG_ENTRY);
        LOG_ENTRY log_entry = {0};
        while(block_position < BLOCK_SIZE - 4 - entry_length){
            memcpy(&log_entry, block_buffer + block_position, sizeof(LOG_ENTRY));
            printf("Time: %d, Throttle Value: %d\n", log_entry.timestamp, log_entry.channel);
            //uint32_t timestamp = *(uint32_t*)(block_buffer + block_position);
            //uint16_t channel   = *(uint16_t*)(block_buffer + block_position + 4);
            //printf("Time: %d, Throttle Value: %d\n", timestamp, channel);
            block_position += entry_length;
        }
    }
}

static void EXPORT_FLIGHT(int chosen_flight){
    
    const int start_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].start_block;
    const int end_block = sd_metadata_block.sd_file_metadata_chunk[FLIGHT_NUM_TO_INDEX(chosen_flight)].end_block;
    uint8_t block_buffer[512] = {0};
    FILE* of2 = fopen("test.csv", "wb");
    for(int i = start_block; i <= end_block; i++){
        READ_SINGLE_BLOCK(block_buffer, i);
        int block_position = 0;
        int entry_length = sizeof(LOG_ENTRY);
        LOG_ENTRY log_entry = {0};
        while(block_position < BLOCK_SIZE - 4 - entry_length){
            memcpy(&log_entry, block_buffer + block_position, sizeof(LOG_ENTRY));
            fprintf(of2, "%d, %d\n", log_entry.timestamp, log_entry.channel);
            block_position += entry_length;
        }
    }
    fclose(of2);
}

int INITIALIZE_SD_CARD(const char* PATH){
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

    int minimum_flight_num = sd_superblock.absolute_flight_num - sd_superblock.relative_flight_num;
    int maximum_flight_num = sd_superblock.relative_flight_num - 1;
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
    if(sd_superblock.magic != SUPERBLOCK_MAGIC){
        perror("Wrong superblock magic number");
        return 1;
    }
    #if VERBOSE_OUTPUT
    printf("Superblock version: %d\nRelative flight number: %d\nMagic number: %08X\n", sd_superblock.version, sd_superblock.relative_flight_num, sd_superblock.magic);
    for(int i = 0; i < sd_superblock.relative_flight_num; i++){
        //printf("Flight %d, read block %d, index %d\n", i, FLIGHT_NUM_TO_BLOCK(i), FLIGHT_NUM_TO_INDEX(i));
        printf("Flight %d, active flag %d, completion flag %d, start block %d, end block %d\n", sd_metadata_block.sd_file_metadata_chunk[i].flight_number, sd_metadata_block.sd_file_metadata_chunk[i].active_flag, sd_metadata_block.sd_file_metadata_chunk[i].log_finished, sd_metadata_block.sd_file_metadata_chunk[i].start_block, sd_metadata_block.sd_file_metadata_chunk[i].end_block);
    }
    #endif
    printf("Flight %d, active flag %d, completion flag %d, start block %d, end block %d\n", sd_metadata_block.sd_file_metadata_chunk[flight_chosen].flight_number, sd_metadata_block.sd_file_metadata_chunk[flight_chosen].active_flag, sd_metadata_block.sd_file_metadata_chunk[flight_chosen].log_finished, sd_metadata_block.sd_file_metadata_chunk[flight_chosen].start_block, sd_metadata_block.sd_file_metadata_chunk[flight_chosen].end_block);
    printf("Dumping data to test.bin...\n");
    DUMP_FLIGHT_TO_BIN(flight_chosen);
    printf("Done.\n");

    printf("Printing Data...\n");
    PRINT_FLIGHT_DATA(flight_chosen);
    printf("\n\nDone.");

    printf("Exporting to test.csv...\n");
    EXPORT_FLIGHT(flight_chosen);
    printf("Done.\n");

    close(fd);
    return 0;
}
