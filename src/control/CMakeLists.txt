cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(IMPLEMENTATION_FILES 
  src/node_/node_control.cpp
  src/pid/pid.cpp
  src/adapter_control/adapter.cpp
  src/adapter_control/fsds.cpp
  src/adapter_control/pac_sim.cpp
  src/adapter_control/eufs.cpp
  src/adapter_control/vehicle.cpp
  src/pure_pursuit/pp.cpp
  src/point_solver/psolver.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common_lib/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pacsim REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node_control src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(node_control rclcpp std_msgs sensor_msgs custom_interfaces fs_msgs message_filters pacsim ackermann_msgs std_srvs tf2_ros tf2_geometry_msgs eufs_msgs)
target_include_directories(node_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(node_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS node_control
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/pid_test.cpp
    test/pp_test.cpp
    test/psolver_test.cpp
    test/node_control_test.cpp
    test/include/utils.cpp
  )
  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})

  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs sensor_msgs fs_msgs custom_interfaces message_filters pacsim ackermann_msgs std_srvs tf2_ros tf2_geometry_msgs eufs_msgs)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
endif()

ament_package()
