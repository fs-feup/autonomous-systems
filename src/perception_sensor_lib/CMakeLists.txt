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


# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(IMPLEMENTATION_FILES
  src/data_association/maximum_likelihood_nll.cpp
  src/data_association/maximum_likelihood_md.cpp
  src/data_association/nearest_neighbour.cpp
  src/data_association/nearest_neighbor.cpp
  src/observation_model/base_observation_model.cpp
)

add_library(${PROJECT_NAME} SHARED ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME} ${PACKAGES})

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
    test/observation_model/base_observation_model_test.cpp
    test/data_association/maximum_likelihood_nll_test.cpp
    test/data_association/nearest_neighbor_test.cpp
    test/data_association/maximum_likelihood_md_test.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES} TIMEOUT 600)
  ament_target_dependencies(${PROJECT_NAME}_test ${PACKAGES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()
  
ament_package()
    