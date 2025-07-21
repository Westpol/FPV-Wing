    #include "log_reader.h"
    #include "stdio.h"
    #include "stdint.h"
    #include <fcntl.h>
    #include <unistd.h>
    #include <string.h>
    #include "stdbool.h"

    uint8_t buffer[512] = {0};
    int fd;
    SD_SUPERBLOCK sd_superblock;

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

    int INITIALIZE_SD_CARD(const char* PATH){
        fd = open(PATH, O_RDONLY);
        if (fd < 0) {
            perror("Failed to open device");
            return 1;
        }

        if (READ_SINGLE_BLOCK(buffer, SUPERBLOCK_INDEX) != 0) {
            fprintf(stderr, "Failed to read superblock\n");
            close(fd);
            return 1;
        }
        memcpy(&sd_superblock, &buffer, sizeof(sd_superblock));
        if(sd_superblock.magic != SUPERBLOCK_MAGIC){
            perror("Wrong superblock magic number");
            return 1;
        }
        printf("Superblock version: %d\nRelative flight number: %d\nMagic number: %08X\n", sd_superblock.version, sd_superblock.relative_flight_num, sd_superblock.magic);
        return 0;
    }
