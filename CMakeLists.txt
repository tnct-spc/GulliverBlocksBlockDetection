cmake_minimum_required(VERSION 3.10.2)
project(gulliver_blocks CXX)
add_definitions("-std=c++11 -lrealsense2")
add_executable(a.out main.cpp communication.cpp detection.cpp manager.cpp)
