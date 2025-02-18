cmake_minimum_required(VERSION 3.8)
project(mocker_node)

# Set C++ standard to 17 and required compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


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
find_package(eufs_msgs REQUIRED)
find_package(common_lib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(IMPLEMENTATION_FILES
  src/node/mocker_node.cpp
  src/utils/mocks.cpp
)

add_executable(mocker_node src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(mocker_node rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs eufs_msgs common_lib visualization_msgs)
target_include_directories(mocker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mocker_node yaml-cpp)
target_compile_features(mocker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mocker_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(custom_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(fs_msgs REQUIRED)
  find_package(eufs_msgs REQUIRED)
  find_package(common_lib REQUIRED)
  find_package(visualization_msgs REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs sensor_msgs fs_msgs eufs_msgs custom_interfaces common_lib visualization_msgs)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(mocker_node_test yaml-cpp)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

ament_package()
