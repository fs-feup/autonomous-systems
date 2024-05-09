cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IMPLEMENTATION_FILES 
  src/node_/node_control.cpp
  src/pid/pid.cpp
  src/adapter_control/adapter.cpp
  src/adapter_control/fsds.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(message_filters REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node_control src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(node_control rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs message_filters)
target_include_directories(node_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS node_control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/pid_test.cpp
  )
  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})

  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs sensor_msgs fs_msgs custom_interfaces message_filters)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif()

ament_package()