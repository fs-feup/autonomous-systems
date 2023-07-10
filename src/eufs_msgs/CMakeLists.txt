cmake_minimum_required(VERSION 3.5)
project(eufs_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List of message files
set(msg_files
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/CanState.msg"
  "msg/CarState.msg"
  "msg/ChassisCommand.msg"
  "msg/ChassisState.msg"
  "msg/ConeArray.msg"
  "msg/ConeArrayWithCovariance.msg"
  "msg/ConeWithCovariance.msg"
  "msg/Costmap.msg"
  "msg/EKFErr.msg"
  "msg/EKFState.msg"
  "msg/FullState.msg"
  "msg/Heartbeat.msg"
  "msg/IntegrationErr.msg"
  "msg/LapStats.msg"
  "msg/NodeState.msg"
  "msg/NodeStateArray.msg"
  "msg/PathIntegralParams.msg"
  "msg/PathIntegralStats.msg"
  "msg/PathIntegralStatus.msg"
  "msg/PathIntegralTiming.msg"
  "msg/PointArray.msg"
  "msg/PointArrayStamped.msg"
  "msg/Runstop.msg"
  "msg/SLAMErr.msg"
  "msg/SLAMState.msg"
  "msg/StateMachineState.msg"
  "msg/SystemStatus.msg"
  "msg/TopicStatus.msg"
  "msg/VehicleCommands.msg"
  "msg/VehicleCommandsStamped.msg"
  "msg/Waypoint.msg"
  "msg/WaypointArrayStamped.msg"
  "msg/WheelOdometryErr.msg"
  "msg/WheelSpeeds.msg"
  "msg/WheelSpeedsStamped.msg"
)

set(action_files
  "action/CheckForObjects.action"
)

set(srv_files
  "srv/Register.srv"
  "srv/SetCanState.srv")

# Generate messages & services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
