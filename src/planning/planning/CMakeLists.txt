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
# sudo apt-get install libcgal-dev libgmp-dev libmpfr-dev

set(GMP_DIR ${CGAL_INSTALLATION_PACKAGE_DIR}/auxiliary/gmp/)
find_package(CGAL REQUIRED Core)
find_package(GMP)
find_package(MPFR)
find_package(Qt5)

add_definitions(-DCGAL_USE_BASIC_VIEWER)

add_executable(planning src/orchestrator.cpp
 src/finalpathplanner.cpp src/slampathplanner.cpp src/cone.cpp src/track.cpp src/position.cpp)
ament_target_dependencies(planning ament_index_cpp rclcpp std_msgs custom_interfaces)

# Link the CGAL library and its dependencies to the target
target_link_libraries(planning ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} ${QT5_LIBRARIES})

target_include_directories(planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(planning PUBLIC c_std_99 cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS
  planning
  DESTINATION lib/${PROJECT_NAME})