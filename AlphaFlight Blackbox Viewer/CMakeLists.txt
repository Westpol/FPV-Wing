cmake_minimum_required(VERSION 3.10)

project(blackbox C)

set(SOURCES
    main.c
    log_reader.c
    logging_structs.c
)

add_executable(blackbox ${SOURCES})
