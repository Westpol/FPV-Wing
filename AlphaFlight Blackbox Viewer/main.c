#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "log_reader.h"

int main(int argc, char *argv[]){
    if (argc != 3) {
        if(argc == 1){
            printf("Use -h for help.\n");
            return 0;
        }
        if(argc > 1){
            if(strcmp(argv[1], "-h") == 0){
                printf("Usage: %s <device/img> <csv_output>\n", argv[0]);
                return 0;
            }
        }
        return 1;
    }
    const char* dev_path = argv[1];
    INITIALIZE_SD_CARD(dev_path);
    return 0;
}

