cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(OSQP_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../ext/osqp/install)
list(PREPEND CMAKE_PREFIX_PATH "${OSQP_INSTALL_DIR}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pacsim REQUIRED)
find_package(common_lib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS registration common io)

set(GMP_DIR ${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/)
find_package(CGAL REQUIRED Core)
find_package(GMP)
find_package(MPFR)
find_package(GSL REQUIRED)

find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

set(IMPLEMENTATION_FILES
  src/adapter_planning/eufs.cpp
  src/adapter_planning/fsds.cpp
  src/adapter_planning/pacsim.cpp
  src/adapter_planning/vehicle.cpp
  src/planning/path_calculation.cpp
  src/planning/smoothing.cpp
  src/planning/planning.cpp
  src/planning/velocity_planning.cpp
  src/planning/skidpad.cpp
  src/planning/midpoint_generator.cpp
  src/planning/colorpoint.cpp
  src/utils/files.cpp
)

set(TESTFILES
  test/main.cpp
  test/integration_tests.cpp
  test/performance_test.cpp
  test/smoothing_tests.cpp
  test/splines_tests.cpp
  test/test_utils/utils.cpp
  test/velocity_planning_tests.cpp
)

add_definitions(-DCGAL_USE_BASIC_VIEWER)
set(CMAKE_BUILD_TYPE "Release")

add_executable(planning src/main.cpp ${IMPLEMENTATION_FILES})

set_target_properties(planning PROPERTIES
  INSTALL_RPATH "${OSQP_INSTALL_DIR}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_target_dependencies(planning
  ament_index_cpp rclcpp std_msgs custom_interfaces eufs_msgs fs_msgs
  tf2_ros tf2_geometry_msgs pacsim common_lib std_srvs nav_msgs
  visualization_msgs
)

target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIRS}
)

target_link_libraries(planning
  ${CGAL_LIBRARIES}
  ${GMP_LIBRARIES}
  ${MPFR_LIBRARIES}
  ${QT5_LIBRARIES}
  ${GSL_LIBRARIES}
  yaml-cpp
  ${PCL_LIBRARIES}
  osqp::osqp
  OsqpEigen::OsqpEigen
)

target_compile_features(planning PUBLIC c_std_99 cxx_std_17)

install(TARGETS planning DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(planning_test ${TESTFILES} ${IMPLEMENTATION_FILES})

  set_target_properties(planning_test PROPERTIES
    INSTALL_RPATH "${OSQP_INSTALL_DIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )

  ament_target_dependencies(planning_test
    ament_index_cpp rclcpp std_msgs custom_interfaces eufs_msgs fs_msgs
    tf2_ros tf2_geometry_msgs pacsim common_lib std_srvs nav_msgs
    visualization_msgs
  )

  target_include_directories(planning_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIRS}
  )

  target_link_libraries(planning_test
    ${CGAL_LIBRARIES}
    ${GMP_LIBRARIES}
    ${MPFR_LIBRARIES}
    ${QT5_LIBRARIES}
    ${GSL_LIBRARIES}
    yaml-cpp
    ${PCL_LIBRARIES}
    osqp::osqp
    OsqpEigen::OsqpEigen
  )

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()