cmake_minimum_required(VERSION 3.8)
project(ros_can)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(canlib REQUIRED)
find_package(std_srvs REQUIRED)


set(IMPLEMENTATION_FILES 
  src/node/node_ros_can.cpp
  src/utils/utils.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${IMPLEMENTATION_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs custom_interfaces canlib)
target_link_libraries(ros_can canlib)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ros_can
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  set(TESTFILES
    test/test_can.cpp
  )

  include_directories(test)  # Include the 'test' directory for your test targets
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Include the current source directory

  # Unit tests
  ament_add_gmock(${PROJECT_NAME}_test ${TESTFILES} ${IMPLEMENTATION_FILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp std_msgs std_srvs custom_interfaces canlib)
  target_link_libraries(${PROJECT_NAME}_test canlib)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS ${PROJECT_NAME}_test DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
