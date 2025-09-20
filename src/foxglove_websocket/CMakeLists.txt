cmake_minimum_required(VERSION 3.8)
project(foxglove_websocket)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found, using header-only approach")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(WEBSOCKETS REQUIRED libwebsockets)

include_directories(include)
include_directories(${WEBSOCKETS_INCLUDE_DIRS})

add_executable(simple_websocket_server
  src/simple_websocket_server.cpp
)

ament_target_dependencies(simple_websocket_server
  rclcpp
)

if(nlohmann_json_FOUND)
  target_link_libraries(simple_websocket_server nlohmann_json::nlohmann_json)
endif()

target_link_libraries(simple_websocket_server ${WEBSOCKETS_LIBRARIES})
target_compile_options(simple_websocket_server PRIVATE ${WEBSOCKETS_CFLAGS_OTHER})

# Topic Relay Client (C++)
add_executable(topic_relay_client
  src/topic_relay_client.cpp
)

ament_target_dependencies(topic_relay_client
  rclcpp
  std_msgs
  geometry_msgs
)

# Link nlohmann_json for topic relay client
if(nlohmann_json_FOUND)
  target_link_libraries(topic_relay_client nlohmann_json::nlohmann_json)
endif()

# Link libwebsockets for topic relay client
target_link_libraries(topic_relay_client ${WEBSOCKETS_LIBRARIES})
target_compile_options(topic_relay_client PRIVATE ${WEBSOCKETS_CFLAGS_OTHER})

install(TARGETS simple_websocket_server topic_relay_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  node.json
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()