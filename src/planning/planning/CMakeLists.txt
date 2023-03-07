cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


add_executable(planning_endurance src/communication/publisher.cpp
 src/pathplanner.cpp src/track.cpp src/position.cpp)
ament_target_dependencies(planning_endurance ament_index_cpp rclcpp std_msgs custom_interfaces)

target_include_directories(planning_endurance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning_endurance PUBLIC c_std_99 cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS
  planning_endurance
  DESTINATION lib/${PROJECT_NAME})