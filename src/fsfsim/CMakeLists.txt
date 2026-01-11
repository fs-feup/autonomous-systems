cmake_minimum_required(VERSION 3.8)
project(fsfsim)

# Set C++ standard to 17 and required compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common_lib REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

set(IMPLEMENTATION_FILES
  src/node/fsfsim_node.cpp
  src/config/config_loader.cpp
  src/vehicle_model/bicycle_model.cpp
)

add_executable(fsfsim src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(fsfsim
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs rosgraph_msgs tf2 tf2_ros
  yaml-cpp common_lib eufs_msgs fs_msgs visualization_msgs custom_interfaces)
target_include_directories(fsfsim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fsfsim yaml-cpp)
target_compile_features(fsfsim PUBLIC c_std_99 cxx_std_17)

install(TARGETS fsfsim
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/test.cpp
  )

  include_directories(test)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros
    eufs_msgs fs_msgs visualization_msgs custom_interfaces)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs rosgraph_msgs tf2 tf2_ros
  yaml-cpp common_lib eufs_msgs fs_msgs visualization_msgs custom_interfaces)

ament_package()