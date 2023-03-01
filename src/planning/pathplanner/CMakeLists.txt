cmake_minimum_required(VERSION 3.17)
project(planning)
include_directories(${CMAKE_SOURCE_DIR})
add_subdirectory(lib\\googletest)
set(CMAKE_CXX_STANDARD 14)

add_executable(planning main.cpp pathplanner.cpp position.cpp track.cpp)