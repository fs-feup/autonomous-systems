cmake_minimum_required(VERSION 3.8)
project(davide_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(davide_node_publisher src/publisher_davide_node.cpp)
ament_target_dependencies(davide_node_publisher rclcpp std_msgs)

add_executable(davide_node_subscriber src/subscriber_davide_node.cpp)
ament_target_dependencies(davide_node_subscriber rclcpp std_msgs)

install(TARGETS
  davide_node_publisher
  davide_node_subscriber
  DESTINATION lib/${PROJECT_NAME})


  

ament_package()
