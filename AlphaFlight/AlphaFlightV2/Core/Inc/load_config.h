/*
 * load_config.h
 *
 *  Created on: Oct 8, 2025
 *      Author: benno
 */

#ifndef INC_LOAD_CONFIG_H_
#define INC_LOAD_CONFIG_H_

#include <stdint.h>

#define CONFIG_HEADER_MAGIC 0xA3D7BC49
#define CONFIG_DATA_MAGIC

typedef struct __attribute((packed)){
	uint32_t magic;
	uint16_t index_next_datastruct;
	uint8_t num_config_blocks;
	uint8_t version;
}CONFIG_HEADER;

typedef struct __attribute((packed)){
	uint32_t magic;
	uint16_t index_next_datastruct;		// sizeof(GYRO_CONFIG)
	int8_t x_direction;
	int8_t y_direction;
	int8_t z_direction;
	float gyro_x_filter;
	float gyro_y_filter;
	float gyro_z_filter;
	uint16_t gyro_polling_rate;
}GYRO_CONFIG_DATA;

void print_block_blocking(uint32_t block);

#endif /* INC_LOAD_CONFIG_H_ */
