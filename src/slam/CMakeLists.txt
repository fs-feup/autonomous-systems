cmake_minimum_required(VERSION 3.8)
project(slam)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -DNDEBUG)
endif()

# find dependencies
set(PACKAGES
    ament_cmake
    rclcpp
    Eigen3
    std_msgs
    sensor_msgs
    custom_interfaces
    eufs_msgs
    std_srvs
    fs_msgs
    pacsim
    geometry_msgs
    common_lib
    visualization_msgs
    message_filters
    motion_lib
    tf2
    tf2_ros
    perception_sensor_lib
    yaml-cpp
)
find_package(GTSAM REQUIRED) # Outside because it is not ament package
foreach(pkg ${PACKAGES})
  find_package(${pkg} REQUIRED)
endforeach()

# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(IMPLEMENTATION_FILES
  src/ros_node/slam_node.cpp
  src/adapter_slam/pacsim.cpp
  src/adapter_slam/vehicle.cpp
  src/slam_config/general_config.cpp
  src/slam_solver/slam_solver.cpp
  src/slam_solver/ekf_slam_solver.cpp
  src/slam_solver/graph_slam_solver/graph_slam_solver.cpp
  src/slam_solver/graph_slam_solver/pose_updater.cpp
  src/slam_solver/graph_slam_solver/graph_slam_instance.cpp
  src/slam_solver/graph_slam_solver/utils.cpp
  src/slam_solver/graph_slam_solver/optimizer/isam2_optimizer.cpp
  src/slam_solver/graph_slam_solver/optimizer/normal_levenberg_optimizer.cpp
  src/slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer.cpp
  src/track_loader/track_loader.cpp
)

add_executable(slam src/main.cpp ${IMPLEMENTATION_FILES})
target_link_libraries(slam tbb gtsam yaml-cpp)
ament_target_dependencies(slam ${PACKAGES})
target_include_directories(slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_features(slam PUBLIC c_std_99 cxx_std_17)

install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/slam_solver/graph_slam_solver/graph_slam_solver_test.cpp
    test/slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer_test.cpp
    test/slam_solver/ekf_slam_solver_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gmock(slam_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  target_link_libraries(slam_test gmock gmock_main tbb gtsam yaml-cpp) # Necessary to be here before Eigen3
  ament_target_dependencies(slam_test ${PACKAGES})
  target_include_directories(slam_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_compile_features(slam_test PUBLIC c_std_99 cxx_std_17)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()