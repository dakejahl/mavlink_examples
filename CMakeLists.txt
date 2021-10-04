cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_examples VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_subdirectory(src/camera)
add_subdirectory(src/winch)
add_subdirectory(src/stress_test)
add_subdirectory(src/receive)
