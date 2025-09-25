cmake_minimum_required(VERSION 3.8)
project(common_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGES
  ament_cmake
  Eigen3
  eufs_msgs
  fs_msgs
  visualization_msgs
  std_msgs
  rclcpp
  geometry_msgs
  custom_interfaces
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

set(IMPLEMENTATION_FILES
  src/competition_logic/mission_logic.cpp
  src/competition_logic/color.cpp
  src/maths/matrixes.cpp
  src/maths/angle_and_norms.cpp
  src/maths/transformations.cpp
  src/structures/position.cpp
  src/structures/pose.cpp
  src/structures/cone.cpp
  src/structures/path_point.cpp
  src/structures/vehicle_state.cpp
  src/structures/velocities.cpp
  src/sensor_data/wheel_encoders.cpp
  src/sensor_data/imu.cpp
  src/communication/marker.cpp
  src/communication/interfaces.cpp
  src/config_load/config_load.cpp
  src/conversions/cones.cpp
  src/car_parameters/car_parameters.cpp
  src/structures/mid_point.cpp
)

add_library(${PROJECT_NAME} SHARED ${IMPLEMENTATION_FILES})
target_link_libraries(${PROJECT_NAME} tbb yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGES})


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

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
    test/maths/transformations_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  target_link_libraries(${PROJECT_NAME}_test tbb yaml-cpp)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
  )

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()
  
ament_package()
    