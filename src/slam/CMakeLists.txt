cmake_minimum_required(VERSION 3.8)
project(slam)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
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
)
foreach(pkg ${PACKAGES})
  find_package(${pkg} REQUIRED)
endforeach()

# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
link_directories(${Eigen_INCLUDE_DIRS})

set(IMPLEMENTATION_FILES
  src/ros_node/slam_node.cpp
  src/adapter_slam/eufs.cpp
  src/adapter_slam/pacsim.cpp
  src/adapter_slam/vehicle.cpp
  src/slam_config/general_config.cpp
  src/slam_solver/slam_solver.cpp
  src/slam_solver/graph_slam/graph_slam_solver.cpp
  src/slam_solver/ekf_slam/ekf_slam_solver.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(slam ${PACKAGES} Eigen3)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/common_lib/structures  
)

target_compile_features(slam PUBLIC c_std_99 cxx_std_17)

include_directories(${EIGEN_INCLUDE_DIR} ${EIGEN_ROOT_DIR})
target_link_libraries(slam ${Eigen_LIBRARIES})

install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    # test/data_association_test.cpp
    # test/motion_model_test.cpp
    # test/observation_model_test.cpp
    # test/slam_test.cpp
    # test/slam_performance_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/common_lib/structures  
  )

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()