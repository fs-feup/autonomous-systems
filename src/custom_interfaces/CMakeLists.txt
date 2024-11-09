cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  "msg/ControlCommand.msg"
  "msg/Point2d.msg"
  "msg/PointArray.msg"
  "msg/VehicleState.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/VcuCommand.msg"
  "msg/PathPoint.msg"
  "msg/PathPointArray.msg"
  "msg/Vcu.msg"
  "msg/Imu.msg"
  "msg/Gps.msg"
  "msg/OperationalStatus.msg"
  "msg/SteeringAngle.msg"
  "msg/WheelRPM.msg"
  "msg/ImuData.msg"
  "msg/EvaluatorControlData.msg"
  "msg/YawPitchRoll.msg"
  "msg/ImuAcceleration.msg"
  "msg/HydraulicLinePressure.msg"
  "msg/MasterLog.msg"
  DEPENDENCIES builtin_interfaces std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
