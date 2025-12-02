cmake_minimum_required(VERSION 3.8)
project(perception_sensor_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

set(PACKAGES
    ament_cmake
    Eigen3
    std_msgs
    rclcpp
    visualization_msgs
    geometry_msgs
    custom_interfaces
    common_lib
    eufs_msgs
    fs_msgs
)

foreach(pkg ${PACKAGES})
  find_package(${pkg} REQUIRED)
endforeach()

# Find PCL
find_package(PCL COMPONENTS registration common io QUIET)
if(NOT PCL_FOUND)
  message(WARNING "PCL not found; building perception_sensor_lib without PCL-dependent features")
else()
endif()

# Enable OpenMP (optional)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(IMPLEMENTATION_FILES
  src/data_association/maximum_likelihood_nll.cpp
  src/data_association/maximum_likelihood_md.cpp
  src/data_association/nearest_neighbour_icp.cpp
  src/data_association/nearest_neighbor.cpp
  src/data_association/jcbb.cpp
  src/observation_model/slam/slam_base_observation_model.cpp
  src/observation_model/ve/no_slip_bicycle_model.cpp
  src/landmark_filter/consecutive_counter_filter.cpp
  src/loop_closure/lap_counter.cpp
)

add_library(${PROJECT_NAME} SHARED ${IMPLEMENTATION_FILES})

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Link libraries & dependencies
ament_target_dependencies(${PROJECT_NAME} ${PACKAGES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

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
    test/observation_model/slam/slam_base_observation_model_test.cpp
    test/observation_model/ve/no_slip_bicycle_model_test.cpp
    test/data_association/maximum_likelihood_nll_test.cpp
    test/data_association/nearest_neighbor_test.cpp
    test/data_association/nearest_neighbour_test.cpp
    test/data_association/maximum_likelihood_md_test.cpp
    test/landmark_filter/consecutive_counter_filter_test.cpp
  )

  include_directories(test)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES})

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
