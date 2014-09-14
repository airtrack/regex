cmake_minimum_required(VERSION 2.6)

project(regex)

set(CMAKE_CXX_FLAGS
    "-std=c++11 -Wall -Wextra -Wno-unused-parameter -O2"
    )

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

add_subdirectory(unittests)
