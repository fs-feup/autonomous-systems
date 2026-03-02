cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
set(PACKAGES
    ament_cmake
    rclcpp
    Eigen3
    std_msgs
    sensor_msgs
    custom_interfaces
    std_srvs
    fs_msgs
    eufs_msgs
    pacsim
    geometry_msgs
    common_lib
    visualization_msgs
    message_filters
    motion_lib
    yaml-cpp
)
foreach(pkg ${PACKAGES})
  find_package(${pkg} REQUIRED)
endforeach()

# Enable OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
link_directories(${Eigen_INCLUDE_DIRS})

set(IMPLEMENTATION_FILES
  src/node/node.cpp
  src/adapters/pacsim.cpp
  src/estimators/ukf.cpp
  src/models/process/component_based_vehicle_model.cpp
  src/models/observation/02_sensors.cpp
  src/models/observation/pacsim_sensors.cpp
  src/utils/parameters.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${IMPLEMENTATION_FILES})
target_link_libraries(${PROJECT_NAME} ${Eigen_LIBRARIES} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGES} Eigen3)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/common_lib/structures  
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)


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
