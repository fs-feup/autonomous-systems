cmake_minimum_required(VERSION 3.8)
project(control)

# ------------------------------------------------------------------------------
# Compiler options 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native -ffast-math)
endif()

# ------------------------------------------------------------------------------
# Paths

# Acados install paths
set(ACADOS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../ext/acados)
set(ACADOS_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../build/acados-install)
set(ACADOS_LIB_DIR ${ACADOS_INSTALL_DIR}/lib)
message(STATUS "Using acados from: ${ACADOS_INSTALL_DIR}")

# REQUIRE acados-install: fail if not present (never use system acados)
if(NOT EXISTS "${ACADOS_INSTALL_DIR}/lib")
  message(FATAL_ERROR 
    "acados not found at: ${ACADOS_INSTALL_DIR}\n"
    "Please run: bash src/control/dependencies_install.sh")
endif()

# Workspace paths
set(WORKSPACE_INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../install)

# The generated solvers bake in the prediction horizon read from these configs,
# so they must be regenerated whenever the configs change - otherwise the solver
# dimensions silently disagree with what the node loads at runtime.
file(GLOB MPC_CONFIG_DEPS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/global/global_config.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/control/*.yaml
)

# Bombated MPC paths
set(ACADOS_BOMBATED_MPC_NAME "bombated_mpc" CACHE STRING "Name of the generated acados MPC")
set(ACADOS_BOMBATED_MPC_GEN_DIR
  ${CMAKE_BINARY_DIR}/control/${ACADOS_BOMBATED_MPC_NAME}/acados/c_generated_code
)
set(ACADOS_BOMBATED_MPC_SOLVER_LIB
  ${ACADOS_BOMBATED_MPC_GEN_DIR}/libacados_ocp_solver_${ACADOS_BOMBATED_MPC_NAME}.so
)

# SuperMPC
set(ACADOS_SUPERMPC_NAME "supermpc" CACHE STRING "Name of the generated acados MPC")
set(ACADOS_SUPERMPC_GEN_DIR
  ${CMAKE_BINARY_DIR}/control/${ACADOS_SUPERMPC_NAME}/acados/c_generated_code
)
set(ACADOS_SUPERMPC_SOLVER_LIB
  ${ACADOS_SUPERMPC_GEN_DIR}/libacados_ocp_solver_${ACADOS_SUPERMPC_NAME}.so
)

# Bombated MPCzinho
set(ACADOS_MPCZINHO_NAME "mpczinho" CACHE STRING "Name of the generated acados MPC")
set(ACADOS_MPCZINHO_GEN_DIR
  ${CMAKE_BINARY_DIR}/control/${ACADOS_MPCZINHO_NAME}/acados/c_generated_code
)
set(ACADOS_MPCZINHO_SOLVER_LIB
  ${ACADOS_MPCZINHO_GEN_DIR}/libacados_ocp_solver_${ACADOS_MPCZINHO_NAME}.so
)

# ------------------------------------------------------------------------------
# Code generation

# Bombated MPC with acados
set(BOMBATED_MPC_ACADOS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/include/solver/bombated_mpc_acados/bombated_mpc_acados.py)
set(BOMBATED_MPC_STAMP ${CMAKE_BINARY_DIR}/control/stamps/bombated_mpc_acados.stamp)
message(STATUS "Will generate acados into build tree: ${ACADOS_BOMBATED_MPC_GEN_DIR}")

add_custom_command(
  OUTPUT ${BOMBATED_MPC_STAMP} ${ACADOS_BOMBATED_MPC_SOLVER_LIB}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${ACADOS_BOMBATED_MPC_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_BOMBATED_MPC_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E env
    AMENT_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{AMENT_PREFIX_PATH}
    CMAKE_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{CMAKE_PREFIX_PATH}
    LD_LIBRARY_PATH=${ACADOS_LIB_DIR}:$ENV{LD_LIBRARY_PATH}
    python3 ${BOMBATED_MPC_ACADOS_SCRIPT}
    --out-dir ${CMAKE_BINARY_DIR}/control/${ACADOS_BOMBATED_MPC_NAME}/acados
    --acados-dir ${ACADOS_INSTALL_DIR}
    --acados-lib-dir ${ACADOS_LIB_DIR}
  COMMAND /bin/sh -c "sed -i -e 's#^INCLUDE_PATH = .*#INCLUDE_PATH = ${ACADOS_INSTALL_DIR}/include#' -e 's#^LIB_PATH = .*#LIB_PATH = ${ACADOS_LIB_DIR}#' ${ACADOS_BOMBATED_MPC_GEN_DIR}/Makefile"
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/control/stamps
  COMMAND ${CMAKE_COMMAND} -E touch ${BOMBATED_MPC_STAMP}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  DEPENDS ${BOMBATED_MPC_ACADOS_SCRIPT} ${MPC_CONFIG_DEPS}
  COMMENT "Generating Bombated MPC Acados solver code..."
)

add_custom_target(generate_bombated_mpc_acados_solver
  DEPENDS ${BOMBATED_MPC_STAMP}
)

# MPCzinho with acados
set(MPCZINHO_ACADOS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/include/solver/mpczinho_acados/mpczinho_acados.py)
set(MPCZINHO_STAMP ${CMAKE_BINARY_DIR}/control/stamps/mpczinho_acados.stamp)
message(STATUS "Will generate acados into build tree: ${ACADOS_MPCZINHO_GEN_DIR}")

add_custom_command(
  OUTPUT ${MPCZINHO_STAMP} ${ACADOS_MPCZINHO_SOLVER_LIB}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${ACADOS_MPCZINHO_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_MPCZINHO_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E env
    AMENT_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{AMENT_PREFIX_PATH}
    CMAKE_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{CMAKE_PREFIX_PATH}
    LD_LIBRARY_PATH=${ACADOS_LIB_DIR}:$ENV{LD_LIBRARY_PATH}
    python3 ${MPCZINHO_ACADOS_SCRIPT}
    --out-dir ${CMAKE_BINARY_DIR}/control/${ACADOS_MPCZINHO_NAME}/acados
    --acados-dir ${ACADOS_INSTALL_DIR}
    --acados-lib-dir ${ACADOS_LIB_DIR}
  COMMAND /bin/sh -c "sed -i -e 's#^INCLUDE_PATH = .*#INCLUDE_PATH = ${ACADOS_INSTALL_DIR}/include#' -e 's#^LIB_PATH = .*#LIB_PATH = ${ACADOS_LIB_DIR}#' ${ACADOS_MPCZINHO_GEN_DIR}/Makefile"
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/control/stamps
  COMMAND ${CMAKE_COMMAND} -E touch ${MPCZINHO_STAMP}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  DEPENDS ${MPCZINHO_ACADOS_SCRIPT} ${MPC_CONFIG_DEPS}
  COMMENT "Generating MPCzinho Acados solver code..."
)

add_custom_target(generate_mpczinho_acados_solver
  DEPENDS ${MPCZINHO_STAMP}
)

# SuperMPC with acados
set(SUPERMPC_ACADOS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/include/solver/supermpc_acados/supermpc_acados.py)
set(SUPERMPC_STAMP ${CMAKE_BINARY_DIR}/control/stamps/supermpc_acados.stamp)

add_custom_command(
  OUTPUT ${SUPERMPC_STAMP} ${ACADOS_SUPERMPC_SOLVER_LIB}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${ACADOS_SUPERMPC_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_SUPERMPC_GEN_DIR}
  COMMAND ${CMAKE_COMMAND} -E env
    AMENT_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{AMENT_PREFIX_PATH}
    CMAKE_PREFIX_PATH=${WORKSPACE_INSTALL_DIR}:$ENV{CMAKE_PREFIX_PATH}
    LD_LIBRARY_PATH=${ACADOS_LIB_DIR}:$ENV{LD_LIBRARY_PATH}
    python3 ${SUPERMPC_ACADOS_SCRIPT}
    --out-dir ${CMAKE_BINARY_DIR}/control/${ACADOS_SUPERMPC_NAME}/acados
    --acados-dir ${ACADOS_INSTALL_DIR}
    --acados-lib-dir ${ACADOS_LIB_DIR}
  COMMAND /bin/sh -c "sed -i -e 's#^INCLUDE_PATH = .*#INCLUDE_PATH = ${ACADOS_INSTALL_DIR}/include#' -e 's#^LIB_PATH = .*#LIB_PATH = ${ACADOS_LIB_DIR}#' ${ACADOS_SUPERMPC_GEN_DIR}/Makefile"
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/control/stamps
  COMMAND ${CMAKE_COMMAND} -E touch ${SUPERMPC_STAMP}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  DEPENDS ${SUPERMPC_ACADOS_SCRIPT} ${MPC_CONFIG_DEPS}
  COMMENT "Generating SuperMPC Acados solver code..."
)

add_custom_target(generate_supermpc_acados_solver
  DEPENDS ${SUPERMPC_STAMP}
)


# ------------------------------------------------------------------------------
# Sources (cpp implementation files)
set(IMPLEMENTATION_FILES
  src/ros_node/ros_node.cpp
  src/adapter/pacsim.cpp
  src/adapter/vehicle.cpp
  src/local_pather/interpolator.cpp
  src/local_pather/alpha_beta.cpp
  src/adapter/invictasim.cpp
  src/lateral_controller/pure_pursuit.cpp
  src/lateral_controller/mpczinho.cpp
  src/longitudinal_controller/pid.cpp
  src/config/parameters.cpp
  src/controller/base_decoupled_controller.cpp
  src/controller/bombated_mpc.cpp
  src/controller/supermpc.cpp
  src/solver/bombated_mpc_acados.cpp
  src/solver/supermpc_acados.cpp
  src/solver/mpczinho_acados.cpp
  src/utils/utils.cpp
  src/utils/path_stager.cpp
)

# ------------------------------------------------------------------------------
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(pacsim REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common_lib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(motion_lib REQUIRED)
find_package(yaml-cpp REQUIRED)

# ------------------------------------------------------------------------------
# Link against acados libraries (acados install and generated solver)

# Generated solver - Bombated MPC
set(ACADOS_BOMBATED_MPC_SOLVER_TARGET acados_ocp_solver_${ACADOS_BOMBATED_MPC_NAME})

add_library(${ACADOS_BOMBATED_MPC_SOLVER_TARGET} SHARED IMPORTED)
set_target_properties(${ACADOS_BOMBATED_MPC_SOLVER_TARGET} PROPERTIES
  IMPORTED_LOCATION ${ACADOS_BOMBATED_MPC_GEN_DIR}/libacados_ocp_solver_${ACADOS_BOMBATED_MPC_NAME}.so
  IMPORTED_NO_SONAME TRUE
)

# Generated solver - SuperMPC
set(ACADOS_SUPERMPC_SOLVER_TARGET acados_ocp_solver_${ACADOS_SUPERMPC_NAME})

add_library(${ACADOS_SUPERMPC_SOLVER_TARGET} SHARED IMPORTED)
set_target_properties(${ACADOS_SUPERMPC_SOLVER_TARGET} PROPERTIES
  IMPORTED_LOCATION ${ACADOS_SUPERMPC_GEN_DIR}/libacados_ocp_solver_${ACADOS_SUPERMPC_NAME}.so
  IMPORTED_NO_SONAME TRUE
)

# Generated solver - MPCzinho
set(ACADOS_MPCZINHO_SOLVER_TARGET acados_ocp_solver_${ACADOS_MPCZINHO_NAME})

add_library(${ACADOS_MPCZINHO_SOLVER_TARGET} SHARED IMPORTED)
set_target_properties(${ACADOS_MPCZINHO_SOLVER_TARGET} PROPERTIES
  IMPORTED_LOCATION ${ACADOS_MPCZINHO_GEN_DIR}/libacados_ocp_solver_${ACADOS_MPCZINHO_NAME}.so
  IMPORTED_NO_SONAME TRUE
)

# acados, hpipm, and blasfeo
foreach(lib acados hpipm blasfeo)
  add_library(${lib} SHARED IMPORTED)
  set_target_properties(${lib} PROPERTIES
    IMPORTED_LOCATION ${ACADOS_LIB_DIR}/lib${lib}.so
  )
endforeach()

set(ACADOS_LIBS
  ${ACADOS_BOMBATED_MPC_SOLVER_TARGET}
  ${ACADOS_MPCZINHO_SOLVER_TARGET}
  ${ACADOS_SUPERMPC_SOLVER_TARGET}
  acados
  hpipm
  blasfeo
  m
)

# ------------------------------------------------------------------------------
# Set include directories
set(CONTROL_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../common_lib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${ACADOS_INSTALL_DIR}/include
  ${ACADOS_SOURCE_DIR}/include
  ${ACADOS_INSTALL_DIR}/include/blasfeo/include
  ${ACADOS_SOURCE_DIR}/external/blasfeo/include
  ${ACADOS_INSTALL_DIR}/include/hpipm/include
  ${ACADOS_SOURCE_DIR}/external/hpipm/include
  ${ACADOS_SOURCE_DIR}/interfaces
  ${ACADOS_BOMBATED_MPC_GEN_DIR}
  ${ACADOS_MPCZINHO_GEN_DIR}
  ${ACADOS_SUPERMPC_GEN_DIR}
)

# ------------------------------------------------------------------------------
# Main executable
add_executable(node_control src/main.cpp ${IMPLEMENTATION_FILES})

add_dependencies(node_control generate_bombated_mpc_acados_solver generate_mpczinho_acados_solver generate_supermpc_acados_solver)

target_include_directories(node_control PUBLIC
  ${CONTROL_INCLUDE_DIRS}
)

ament_target_dependencies(node_control
  rclcpp
  std_msgs
  sensor_msgs
  custom_interfaces  
  message_filters
  pacsim
  ackermann_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  common_lib
  visualization_msgs
  motion_lib
)

target_link_libraries(node_control
  yaml-cpp
  ${ACADOS_LIBS}
)

target_compile_features(node_control PUBLIC c_std_99 cxx_std_17)

set_target_properties(node_control PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ACADOS_LIB_DIR};${ACADOS_BOMBATED_MPC_GEN_DIR};${ACADOS_MPCZINHO_GEN_DIR};${ACADOS_SUPERMPC_GEN_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

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

  add_dependencies(${PROJECT_NAME}_test generate_bombated_mpc_acados_solver generate_mpczinho_acados_solver generate_supermpc_acados_solver)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    ${CONTROL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    sensor_msgs    
    custom_interfaces
    message_filters
    pacsim
    ackermann_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    common_lib
    visualization_msgs
    motion_lib
  )

  target_link_libraries(${PROJECT_NAME}_test
    yaml-cpp
    ${ACADOS_LIBS}
  )

  set_target_properties(${PROJECT_NAME}_test PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${ACADOS_LIB_DIR};${ACADOS_BOMBATED_MPC_GEN_DIR};${ACADOS_MPCZINHO_GEN_DIR};${ACADOS_SUPERMPC_GEN_DIR}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
  )
endif()

ament_package()
