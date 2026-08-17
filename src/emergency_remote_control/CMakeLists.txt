cmake_minimum_required(VERSION 3.8)
project(emergency_remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native -ffast-math)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)

add_library(remote_control_lib
  src/remote_control/remote_control.cpp
)

target_include_directories(remote_control_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(remote_control_lib
  rclcpp
  std_msgs
  custom_interfaces
)

target_link_libraries(remote_control_lib
  Boost::system
  Boost::thread
  Threads::Threads
)

target_compile_features(remote_control_lib PUBLIC cxx_std_17)

add_executable(node_remote_control src/remote_control_main.cpp)
target_link_libraries(node_remote_control remote_control_lib)
target_compile_features(node_remote_control PUBLIC cxx_std_17)

install(TARGETS node_remote_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/remote_control_test.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  target_link_libraries(${PROJECT_NAME}_test
    remote_control_lib
  )
endif()

ament_package()
