/*
 * sd_logger.c
 *
 *  Created on: May 6, 2025
 *      Author: benno
 */

#include "sd_logger.h"
#include "fatfs.h"

static FATFS fs;
static FRESULT res;
static FIL file;
