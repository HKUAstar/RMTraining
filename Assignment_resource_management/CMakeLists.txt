cmake_minimum_required(VERSION 3.21)
project(cpp_training)

set(CMAKE_CXX_STANDARD 23)
set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# Google Test
add_subdirectory(lib/googletest)
include_directories(lib/googletest/googletest/include)
link_libraries(gtest_main)
enable_testing()

# Source
include_directories(include)
add_subdirectory(src/custom_string)
add_subdirectory(src/custom_barrier)
add_subdirectory(src/safe_hashmap)