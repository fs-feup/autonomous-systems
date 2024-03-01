cmake_minimum_required(VERSION 3.8)
project(loc_map)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

#Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
link_directories(${Eigen_INCLUDE_DIRS})

set(IMPLEMENTATION_FILES 
  src/loc_map/lm_node.cpp
  src/kalman_filter/ekf.cpp
  src/kalman_filter/motion_models.cpp
  src/kalman_filter/observation_models.cpp
  src/utils/position.cpp
  src/utils/formulas.cpp
  src/utils/color.cpp
  src/adapter/adapter.cpp
  src/adapter/eufs.cpp
  src/adapter/fsds.cpp
)


add_executable(${PROJECT_NAME} src/loc_map.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(loc_map rclcpp std_msgs sensor_msgs eufs_msgs fs_msgs custom_interfaces Eigen3)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(loc_map PUBLIC c_std_99 cxx_std_17)

include_directories(${EIGEN_INCLUDE_DIR} ${EIGEN_ROOT_DIR})
target_link_libraries(loc_map ${Eigen_LIBRARIES})

install(TARGETS loc_map
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(custom_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(eufs_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(fs_msgs REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/integration_test.cpp
    test/ekf_steps_exec_time.cpp
    test/motion_model_test.cpp
    test/observation_model_test.cpp
    test/slam_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs eufs_msgs sensor_msgs fs_msgs custom_interfaces)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()