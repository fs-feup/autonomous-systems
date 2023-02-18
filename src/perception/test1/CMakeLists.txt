cmake_minimum_required(VERSION 3.9)
project(test1)

set(CMAKE_CXX_STANDARD 11)

#find_library(OpenCV)
find_package(OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 11)

add_executable(test1 main.cpp)

target_link_libraries(test1 ${OpenCV_LIBS})
