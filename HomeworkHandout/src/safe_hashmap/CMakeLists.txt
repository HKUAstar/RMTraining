cmake_minimum_required(VERSION 3.21)
project(safe_hashmap)

set(CMAKE_CXX_STANDARD 23)

# Memory sanitizer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined,address -g")

add_executable(${PROJECT_NAME}
        safe_hashmap.cpp
        unittest.cpp
)
