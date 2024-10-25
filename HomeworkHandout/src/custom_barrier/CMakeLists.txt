cmake_minimum_required(VERSION 3.21)
project(custom_barrier)

set(CMAKE_CXX_STANDARD 23)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined,address -g")

add_executable(${PROJECT_NAME}
        ${PROJECT_NAME}.cpp
        unittest.cpp
        kmeans.cpp
)
