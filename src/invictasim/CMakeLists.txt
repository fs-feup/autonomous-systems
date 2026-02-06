cmake_minimum_required(VERSION 3.8)
project(invictasim)

# =====================
# Compiler setup
# =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(common_lib REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# =====================
# Core library
# =====================
add_library(invictasim_core
  src/loader/config.cpp
  src/vehicle_model/Bicycle/bicycle.cpp
  src/vehicle_model/FSFEUP02/FSFEUP02.cpp
  src/vehicle_model/FSFEUP02/tire/tire.cpp
  src/vehicle_model/FSFEUP02/powertrain/battery/battery.cpp
  src/vehicle_model/FSFEUP02/powertrain/motor/motor.cpp
  src/vehicle_model/FSFEUP02/powertrain/powertrain.cpp
)

ament_target_dependencies(invictasim_core
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  tf2
  tf2_ros
  yaml-cpp
  common_lib
  eufs_msgs
  fs_msgs
  visualization_msgs
  custom_interfaces
)

target_include_directories(invictasim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# =====================
# Main executable
# =====================
add_executable(invictasim
  src/main.cpp
  src/node/invictasim_node.cpp
)

target_link_libraries(invictasim
  invictasim_core
  yaml-cpp
)

install(TARGETS invictasim
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Tests
# =====================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/main.cpp
    test/test.cpp
  )

  target_link_libraries(${PROJECT_NAME}_test
    invictasim_core
    yaml-cpp
  )

  install(TARGETS ${PROJECT_NAME}_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# =====================
# Install & export
# =====================
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  tf2
  tf2_ros
  yaml-cpp
  common_lib
  eufs_msgs
  fs_msgs
  visualization_msgs
  custom_interfaces
)

ament_package()
