cmake_minimum_required(VERSION 3.10)

project(blackbox C)

set(SOURCES
    main.c
    log_reader.c
    logging_structs.c
)

add_executable(blackbox ${SOURCES})

# Enable all warnings and treat them as errors
if (CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
    target_compile_options(blackbox PRIVATE -Wall -Wextra -Wpedantic -Werror)
elseif (CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    # For MSVC compiler
    target_compile_options(blackbox PRIVATE /W4 /WX)
endif()
