cmake_minimum_required(VERSION 3.10)

project(ass3)

set(CMAKE_CXX_STANDARD  17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

add_executable(ass3 ./src/ass3.cpp)

set_target_properties(ass3 PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin/
)

target_link_libraries(ass3 ${OpenCV_LIBS})