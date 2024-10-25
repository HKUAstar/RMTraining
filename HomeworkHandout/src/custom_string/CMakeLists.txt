cmake_minimum_required(VERSION 3.21)
project(custom_string)

set(CMAKE_CXX_STANDARD 23)

# Memory sanitizer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined,address -g")

add_executable(${PROJECT_NAME}
        custom_string.cpp
        unittest.cpp
)
add_executable(test_s
        custom_string.cpp
        main.cpp
)
