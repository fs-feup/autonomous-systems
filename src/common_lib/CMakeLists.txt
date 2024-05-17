cmake_minimum_required(VERSION 3.8)
project(common_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)

# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(IMPLEMENTATION_FILES
  src/competition_logic/mission_logic.cpp
  src/competition_logic/color.cpp
  src/maths/matrixes.cpp
  src/maths/transformations.cpp
  src/structures/position.cpp
  src/structures/pose.cpp
  src/structures/cone.cpp
  src/structures/vehicle_state.cpp
  src/sensor_data/wheel_encoders.cpp
  src/sensor_data/imu.cpp
  src/vehicle_dynamics/bicycle_model.cpp
)

add_library(${PROJECT_NAME} SHARED ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME} eufs_msgs fs_msgs Eigen3)
link_directories(${Eigen_INCLUDE_DIRS})
target_link_libraries(common_lib ${Eigen_LIBRARIES})

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
    test/vehicle_dynamics_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  ament_target_dependencies(${PROJECT_NAME}_test eufs_msgs fs_msgs Eigen3)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()
  
ament_package()
    