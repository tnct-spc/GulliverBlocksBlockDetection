cmake_minimum_required(VERSION 3.10.2)
project(gulliver_blocks CXX)
add_definitions("-std=c++11 -g")
add_executable(a.out main.cpp communication.cpp detection.cpp manager.cpp )
find_package(PkgConfig)

pkg_check_modules(RealSense2 REQUIRED realsense2)
pkg_check_modules(OpenCV REQUIRED opencv)

include_directories(${RealSense2_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(a.out ${RealSense2_LIBRARIES})
target_link_libraries(a.out ${OpenCV_LIBRARIES})

