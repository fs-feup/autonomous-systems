cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(PCL REQUIRED QUIET)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(common_lib REQUIRED)


set(IMPLEMENTATION_FILES 
  src/ground_removal/ransac.cpp
  src/ground_removal/grid_ransac.cpp
  src/cone_differentiation/least_squares_differentiation.cpp
  src/clustering/dbscan.cpp
  src/utils/cluster.cpp
  src/cone_validator/cylinder_validator.cpp
  src/cone_validator/height_validator.cpp
  src/cone_validator/displacement_validator.cpp
  src/cone_validator/deviation_validator.cpp
  src/cone_validator/z_score_validator.cpp
  src/utils/plane.cpp
  src/cone_evaluator/distance_predict.cpp
  src/center_calculation/centroid_calculation.cpp
  src/icp/icp.cpp
  src/center_calculation/circunference_center_calculation.cpp
)

set(TEST_INCLUDE_FILES
  test/ransac_test.cpp
  test/least_squares_test.cpp
  test/dbscan_test.cpp
  test/performance_test.cpp
  test/height_validator_test.cpp
  test/cylinder_validator_test.cpp
  test/displacement_validator_test.cpp
  test/grid_ransac_test.cpp
  test/distance_validator_test.cpp
  test/icp_test.cpp
  test/circunference_center_calculation_test.cpp
  test/standard_deviation_validator_test.cpp
  test/z_score_validator_test.cpp
)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_CONVERSIONS_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(perception src/perception.cpp src/perception/perception_node.cpp ${IMPLEMENTATION_FILES})
target_include_directories(perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${pcl_conversions_INCLUDE_DIRS})
target_compile_features(perception PUBLIC c_std_99 cxx_std_17)

target_link_libraries(perception ${PCL_LIBRARIES})

install(TARGETS perception
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_target_dependencies(perception rclcpp visualization_msgs common_lib custom_interfaces sensor_msgs fs_msgs message_filters eufs_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(perception_test
                  ${IMPLEMENTATION_FILES}
                  ${TEST_INCLUDE_FILES})


  target_link_libraries(perception_test ${PCL_LIBRARIES})
  target_include_directories(perception_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> ${pcl_conversions_INCLUDE_DIRS})

    ament_target_dependencies(perception_test rclcpp custom_interfaces fs_msgs ament_cmake_gtest)

endif()


ament_export_include_directories(include)
ament_package()
