cmake_minimum_required(VERSION 3.8)
project(experiment1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(${PROJECT_NAME} src/node1.cpp src/car.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
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
    test/car_test.cpp
    )
  
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  install(TARGETS
    ${PROJECT_NAME}_test
    # Install the generated test executable file to the path after DESTINATION
    DESTINATION lib/${PROJECT_NAME})

  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
