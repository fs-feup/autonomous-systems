
cmake_minimum_required(VERSION 3.8)
project(invictasim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
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
find_package(motion_lib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

# Implementation files
set(IMPLEMENTATION_FILES
  src/simulator/invictasim.cpp
  src/io/input/input_adapter.cpp
  src/io/input/ros.cpp
  src/io/output/ros.cpp
  src/io/input/keyboard.cpp
  src/config/config.cpp
  src/vehicle_model/FSFEUP02.cpp
  src/vehicle_model/state_est_model.cpp
  src/vehicle_model/rk4_state_est_model.cpp
  src/track/track.cpp
)

# Main executable
add_executable(invictasim src/main.cpp ${IMPLEMENTATION_FILES})
target_include_directories(invictasim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_TTF_INCLUDE_DIRS}
)
target_compile_features(invictasim PUBLIC c_std_99 cxx_std_17)
target_compile_options(invictasim PUBLIC ${SDL2_CFLAGS_OTHER} ${SDL2_TTF_CFLAGS_OTHER})
target_link_libraries(invictasim yaml-cpp ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})
ament_target_dependencies(invictasim
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
  visualization_msgs
  custom_interfaces
  motion_lib
)

install(TARGETS invictasim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(invictasim_test
    test/main.cpp
    test/test.cpp
    ${IMPLEMENTATION_FILES}
  )
  target_link_libraries(invictasim_test yaml-cpp)
  target_include_directories(invictasim_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SDL2_INCLUDE_DIRS}
    ${SDL2_TTF_INCLUDE_DIRS}
  )
  target_compile_options(invictasim_test PUBLIC ${SDL2_CFLAGS_OTHER} ${SDL2_TTF_CFLAGS_OTHER})
  target_link_libraries(invictasim_test ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})
  ament_target_dependencies(invictasim_test
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
    visualization_msgs
    custom_interfaces
    motion_lib
  )
  install(TARGETS invictasim_test
    DESTINATION lib/${PROJECT_NAME})
endif()

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
  visualization_msgs
  custom_interfaces
)
ament_package()
