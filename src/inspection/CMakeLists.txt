cmake_minimum_required(VERSION 3.8)
project(inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(common_lib REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

# Source files
set(IMPLEMENTATION_FILES
  src/inspection_node/inspection_ros.cpp
  src/functions/inspection_functions.cpp
)

# Create executable
add_executable(inspection src/inspection.cpp ${IMPLEMENTATION_FILES})

# Declare dependencies
ament_target_dependencies(inspection rclcpp common_lib eufs_msgs fs_msgs std_msgs std_srvs message_filters custom_interfaces)

# Include directories
target_include_directories(inspection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${message_filters_LIBRARIES})

# Require C++ standards
target_compile_features(inspection PUBLIC c_std_99 cxx_std_17)

# Install executable
install(TARGETS inspection DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Testing (if enabled)
if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  # Test source files
  set(TESTFILES
    test/main.cpp
    test/tests.cpp
  )

  # Include directories for testing
  include_directories(test)  # Include the 'test' directory
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Create the test target
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})

  # Declare test dependencies
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp common_lib eufs_msgs fs_msgs message_filters std_msgs std_srvs custom_interfaces)

  # Include directories for the test target
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> ${message_filters_LIBRARIES})

  # Install the test target
  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

# Generate package metadata
ament_package()
