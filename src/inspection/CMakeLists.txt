cmake_minimum_required(VERSION 3.8)
project(inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

set(IMPLEMENTATION_FILES
  src/inspection_ros.cpp
  src/inspection_functions.cpp
)

add_executable(inspection src/inspection.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(inspection rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs)
target_include_directories(inspection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(inspection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS inspection
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

set(TESTFILES 
  test/main.cpp
  test/tests.cpp
)

include_directories(test)  # Include the 'test' directory for your test targets
include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory
ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs sensor_msgs fs_msgs custom_interfaces)
target_include_directories(${PROJECT_NAME}_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
