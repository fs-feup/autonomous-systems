cmake_minimum_required(VERSION 3.8)
project(ros_can)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(canlib REQUIRED)


set(IMPLEMENTATION_FILES 
  src/node/node_ros_can.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs canlib)
target_link_libraries(ros_can canlib)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ros_can
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
