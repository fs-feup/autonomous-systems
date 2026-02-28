cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_lib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)

set(IMPLEMENTATION_FILES
  src/fov_trimming/fov_trimming.cpp
  src/fov_trimming/cut_trimming.cpp
  src/fov_trimming/acceleration_trimming.cpp
  src/fov_trimming/skidpad_trimming.cpp
  src/ground_removal/ransac.cpp
  src/ground_removal/himmelsbach.cpp
  src/wall_removal/grid_removal.cpp
  src/cone_differentiation/least_squares_differentiation.cpp
  src/clustering/dbscan.cpp
  src/clustering/grid_clustering.cpp
  src/cone_validator/npoints_validator.cpp
  src/cone_validator/cylinder_validator.cpp
  src/cone_validator/height_validator.cpp
  src/cone_validator/displacement_validator.cpp
  src/cone_validator/deviation_validator.cpp
  src/cone_validator/z_score_validator.cpp
  src/deskew/deskew.cpp
  src/utils/plane.cpp
  src/utils/ground_grid.cpp
  src/utils/cluster.cpp
  src/utils/grid_geometry.cpp
  src/cone_evaluator/cone_evaluator.cpp
  src/center_calculation/centroid_calculation.cpp
  src/center_calculation/circunference_center_calculation.cpp
)

set(TEST_INCLUDE_FILES
  test/center_calculation/circunference_center_calculation_test.cpp
  test/fov_trimming/cut_trimming_test.cpp
  test/fov_trimming/skidpad_trimming_test.cpp
  test/fov_trimming/acceleration_trimming_test.cpp
  test/clustering/grid_test.cpp
  test/cone_evaluator/cone_evaluator_test.cpp
  test/ground_removal/ransac_test.cpp
  test/ground_removal/himmelsbach_test.cpp
)

include_directories(${sensor_msgs_INCLUDE_DIRS})

add_executable(perception src/perception.cpp src/perception/perception_node.cpp ${IMPLEMENTATION_FILES})
target_include_directories(perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(perception PUBLIC c_std_99 cxx_std_17)

target_link_libraries(perception yaml-cpp)

install(TARGETS perception
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_target_dependencies(perception rclcpp visualization_msgs common_lib custom_interfaces sensor_msgs fs_msgs message_filters eufs_msgs std_srvs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(common_lib REQUIRED)
  ament_add_gtest(perception_test
                  src/perception/perception_node.cpp
                  ${IMPLEMENTATION_FILES}
                  ${TEST_INCLUDE_FILES})

  target_link_libraries(perception_test yaml-cpp)
  target_include_directories(perception_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(perception_test
      rclcpp
      visualization_msgs
      common_lib
      custom_interfaces
      fs_msgs
      eufs_msgs
      message_filters
      tf2
      tf2_ros
      ament_cmake_gtest
      std_srvs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp visualization_msgs sensor_msgs tf2 tf2_ros custom_interfaces)
ament_package()
