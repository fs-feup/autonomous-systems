cmake_minimum_required(VERSION 3.8)
project(loc_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# custom_interfaces 
find_package(custom_interfaces REQUIRED)

add_executable(loc_map src/loc_map.cpp)
ament_target_dependencies(loc_map rclcpp std_msgs custom_interfaces)
target_include_directories(loc_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(loc_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS loc_map
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES 
    test/main.cpp
    test/base_test.cpp
    )
  
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_include_directories(loc_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS
    ${PROJECT_NAME}_test
    # Install the generated test executable file to the path after DESTINATION
    DESTINATION lib/${PROJECT_NAME})

  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
