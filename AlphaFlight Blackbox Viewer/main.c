#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "log_reader.h"

int main(int argc, char *argv[]){
    if(argc == 1){
        printf("Use -h for help.\n");
        return 0;
    }
    if(argc > 1){
        if(strcmp(argv[1], "-h") == 0){
            printf("Example usage: sudo %s <dev/sdX> -print_data -csv <csv_output.csv> -bin <flight_in_binary.bin>\n", argv[0]);
            return 0;
        }
    }

    const char* file_dir = argv[1];
    bool bin_enable = false;
    int bin_dir_index = 0;
    bool csv_enable = false;
    int csv_dir_index = 0;
    bool print_enable = false;
    
    const char* commands[] = {"-print_data", "-csv", "-bin"};

    for(int i = 1; i < argc; i++){
        for(int f = 0; f < 3; f++){
            if(strcmp(argv[i], commands[f]) == 0){
                if(f == 2 && i + 1 < argc){
                    bin_dir_index = i + 1;
                    bin_enable = true;
                }
                if(f == 1 && i + 1 < argc){
                    csv_dir_index = i + 1;
                    csv_enable = true;
                }
                if(f == 0){
                    print_enable = true;
                }
            }
        }
    }


    INITIALIZE_SD_CARD(file_dir, bin_enable, argv[bin_dir_index], csv_enable, argv[csv_dir_index], print_enable);
    return 0;
}

