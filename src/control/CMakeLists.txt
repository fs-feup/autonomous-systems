cmake_minimum_required(VERSION 3.8)
project(control)

# ------------------------------------------------------------------------------
# Compiler options
# ------------------------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ------------------------------------------------------------------------------
# Paths
# ------------------------------------------------------------------------------
set(ACADOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../ext/acados)
set(ACADOS_LIB_DIR ${ACADOS_DIR}/lib)
set(ACADOS_GEN_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include/solver/acados/c_generated_code
)

# ------------------------------------------------------------------------------
# Generate ACADOS solver code - only runs if acados.py changes
# ------------------------------------------------------------------------------
set(ACADOS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/include/solver/acados/acados.py)
set(ACADOS_GENERATED_MARKER ${ACADOS_GEN_DIR}/.generated)

file(MAKE_DIRECTORY ${ACADOS_GEN_DIR})

add_custom_command(
  OUTPUT ${ACADOS_GENERATED_MARKER}
  COMMAND python3 ${ACADOS_SCRIPT}
  COMMAND ${CMAKE_COMMAND} -E touch ${ACADOS_GENERATED_MARKER}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  DEPENDS ${ACADOS_SCRIPT}
  COMMENT "Generating ACADOS solver code..."
)

add_custom_target(generate_acados_solver DEPENDS ${ACADOS_GENERATED_MARKER})


# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------
set(IMPLEMENTATION_FILES
  src/ros_node/ros_node.cpp
  src/adapter/pacsim.cpp
  src/adapter/eufs.cpp
  src/adapter/vehicle.cpp
  src/lateral_controller/pure_pursuit.cpp
  src/longitudinal_controller/pid.cpp
  src/config/parameters.cpp
  src/controller/base_decoupled_controller.cpp
  src/controller/mpc.cpp
  src/solver/acados.cpp
  src/utils/utils.cpp
)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
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
find_package(common_lib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(motion_lib REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------
# Imported acados libraries
# ------------------------------------------------------------------------------
add_library(acados_ocp_solver_mpc SHARED IMPORTED)
set_target_properties(acados_ocp_solver_mpc PROPERTIES
  IMPORTED_LOCATION ${ACADOS_GEN_DIR}/libacados_ocp_solver_mpc.so
)

foreach(lib acados hpipm blasfeo)
  add_library(${lib} SHARED IMPORTED)
  set_target_properties(${lib} PROPERTIES
    IMPORTED_LOCATION ${ACADOS_LIB_DIR}/lib${lib}.so
  )
endforeach()

set(ACADOS_LIBS
  acados_ocp_solver_mpc
  acados
  hpipm
  blasfeo
  m
)

# ------------------------------------------------------------------------------
# Common include directories
# ------------------------------------------------------------------------------
set(CONTROL_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_lib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/external/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  ${ACADOS_DIR}/external/hpipm/include
  ${ACADOS_DIR}/interfaces/acados_c
  ${ACADOS_GEN_DIR}
)

# ------------------------------------------------------------------------------
# Main executable
# ------------------------------------------------------------------------------
add_executable(node_control src/main.cpp ${IMPLEMENTATION_FILES})

add_dependencies(node_control generate_acados_solver)

target_include_directories(node_control PUBLIC
  ${CONTROL_INCLUDE_DIRS}
)

ament_target_dependencies(node_control
  rclcpp
  std_msgs
  sensor_msgs
  custom_interfaces
  fs_msgs
  message_filters
  pacsim
  ackermann_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  eufs_msgs
  common_lib
  visualization_msgs
  motion_lib
)

target_link_libraries(node_control
  yaml-cpp
  ${ACADOS_LIBS}
)

target_compile_features(node_control PUBLIC c_std_99 cxx_std_17)

install(TARGETS node_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
    test/main.cpp
    test/longitudinal_controller/pid_test.cpp
    test/lateral_controller/pure_pursuit_test.cpp
    test/utils/utils_test.cpp
  )

  ament_add_gtest(${PROJECT_NAME}_test
    ${TESTFILES}
    ${IMPLEMENTATION_FILES}
  )

  add_dependencies(${PROJECT_NAME}_test generate_acados_solver)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ${CONTROL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    sensor_msgs
    fs_msgs
    custom_interfaces
    message_filters
    pacsim
    ackermann_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    eufs_msgs
    common_lib
    visualization_msgs
    motion_lib
  )

  target_link_libraries(${PROJECT_NAME}_test
    yaml-cpp
    ${ACADOS_LIBS}
  )
endif()

ament_package()
