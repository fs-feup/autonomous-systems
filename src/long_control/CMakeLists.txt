cmake_minimum_required(VERSION 3.8)
project(long_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IMPLEMENTATION_FILES 
  src/main.cpp
  src/node_/node_long_control.cpp
  src/adapter/adapter.cpp
  src/adapter/fsds.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fs_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node_long_control ${IMPLEMENTATION_FILES})
ament_target_dependencies(node_long_control rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs)
target_include_directories(node_long_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_long_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS node_long_control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
