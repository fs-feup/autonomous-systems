enable_testing()
# Using googletest for unit tests
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)


find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


FetchContent_MakeAvailable(googletest)

add_executable(unit_tests
  UnitTests.cpp
  ../../src/VehicleModel/VehicleModelBicycle.cpp
  ../../src/transform.cpp
)

target_include_directories(unit_tests PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(unit_tests rclcpp std_srvs visualization_msgs geometry_msgs tf2_ros yaml-cpp sensor_msgs Eigen3)

target_link_libraries(unit_tests 
gtest_main
yaml-cpp)

include(GoogleTest)
gtest_discover_tests(unit_tests)