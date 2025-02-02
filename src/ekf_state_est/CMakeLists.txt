cmake_minimum_required(VERSION 3.8)
project(ekf_state_est)
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
    yaml-cpp
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
  src/ros_node/se_node.cpp
  src/kalman_filter/ekf.cpp
  src/kalman_filter/motion_models.cpp
  src/kalman_filter/observation_models.cpp
  src/kalman_filter/data_association.cpp
  src/adapter_ekf_state_est/adapter.cpp
  src/adapter_ekf_state_est/eufs.cpp
  src/adapter_ekf_state_est/fsds.cpp
  src/adapter_ekf_state_est/pacsim.cpp
  src/adapter_ekf_state_est/vehicle.cpp
)


add_executable(${PROJECT_NAME} src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(ekf_state_est ${PACKAGES} Eigen3)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/common_lib/structures  
)

target_compile_features(ekf_state_est PUBLIC c_std_99 cxx_std_17)

include_directories(${EIGEN_INCLUDE_DIR} ${EIGEN_ROOT_DIR})
target_link_libraries(ekf_state_est ${Eigen_LIBRARIES} yaml-cpp)

install(TARGETS ekf_state_est
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/data_association_test.cpp
    test/motion_model_test.cpp
    test/observation_model_test.cpp
    test/slam_test.cpp
    test/ekf_state_est_performance_test.cpp
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
  target_link_libraries(${PROJECT_NAME}_test ${Eigen_LIBRARIES} yaml-cpp)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()