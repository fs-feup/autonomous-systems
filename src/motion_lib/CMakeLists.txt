cmake_minimum_required(VERSION 3.8)
project(motion_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(common_lib REQUIRED)

# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(IMPLEMENTATION_FILES
  src/vel_process_model/particle_model.cpp
  src/v2p_models/constant_velocity_model.cpp
  src/v2p_models/base_v2p_motion_model.cpp
  src/s2v_model/bicycle_model.cpp
  src/s2v_model/no_rear_wss_bicycle_model.cpp
  src/v2p_models/constant_velocity_turnrate_model.cpp
)

add_library(${PROJECT_NAME} SHARED ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME} eufs_msgs std_msgs rclcpp visualization_msgs fs_msgs Eigen3 custom_interfaces common_lib)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )
  
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  )

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/s2v_models/bicycle_model_test.cpp
    test/v2p_models/constant_velocity_model_test.cpp
    test/v2p_models/constant_velocity_turnrate_model_test.cpp
    test/vel_process_model/particle_model_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  ament_target_dependencies(${PROJECT_NAME}_test eufs_msgs rclcpp fs_msgs std_msgs visualization_msgs Eigen3 custom_interfaces common_lib)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()
  
ament_package()
    