cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
# sudo apt-get install libcgal-dev libgmp-dev libmpfr-dev

set(GMP_DIR ${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/)
find_package(CGAL REQUIRED Core)
find_package(GMP)
find_package(MPFR)
find_package(GSL REQUIRED)

# find_package(Qt5)

set(TESTFILES
  test/main.cpp
  test/tests.cpp
  test/integration_tests.cpp
)

set(IMPLEMENTATION_FILES
  src/adapter/adapter.cpp
  src/planning/global_path_planner.cpp
  src/planning/local_path_planner.cpp
  src/planning/track.cpp
  src/planning/planning.cpp
  src/planning/path_smoothing.cpp
  src/utils/cone.cpp
  src/utils/pathpoint.cpp
  src/utils/files.cpp
)

add_definitions(-DCGAL_USE_BASIC_VIEWER)
set(CMAKE_BUILD_TYPE "Release")

add_executable(planning src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(planning ament_index_cpp rclcpp std_msgs custom_interfaces eufs_msgs fs_msgs)

include_directories(${GSL_INCLUDE_DIRS})
# Link the CGAL library and its dependencies to the target
target_link_libraries(planning ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${QT5_LIBRARIES} ${GSL_LIBRARIES})

target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning PUBLIC c_std_99 cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  ament_add_gtest(planning_test ${TESTFILES} ${IMPLEMENTATION_FILES})
  ament_target_dependencies(planning_test ament_index_cpp rclcpp std_msgs custom_interfaces eufs_msgs fs_msgs)
  target_include_directories(planning_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(planning_test ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${QT5_LIBRARIES} ${GSL_LIBRARIES})

  install(TARGETS
    planning
    planning_test
    DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

