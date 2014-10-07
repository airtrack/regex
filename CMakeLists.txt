cmake_minimum_required(VERSION 2.6)

project(regex)

set(CMAKE_CXX_FLAGS
    "-std=c++11 -Wall -Wextra -Wno-unused-parameter -O2"
    )

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")

add_definitions(-DBENCHMARK_TEST_FILE=\"${PROJECT_SOURCE_DIR}/benchmark/mtent12.txt\")

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

add_subdirectory(benchmark)
add_subdirectory(unittests)
