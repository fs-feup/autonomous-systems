cmake_minimum_required(VERSION 3.8)
project(helper_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eufs_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(cone_publisher src/cone_publisher.cpp)
add_executable(wheel_speeds_publisher src/wss_publisher.cpp)

ament_target_dependencies(cone_publisher rclcpp eufs_msgs)
ament_target_dependencies(wheel_speeds_publisher rclcpp eufs_msgs)

install(TARGETS
  cone_publisher
  wheel_speeds_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()