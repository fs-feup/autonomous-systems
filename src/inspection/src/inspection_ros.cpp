#include "include/inspection_ros.hpp"

#include "exceptions/invalid_mission_exception.hpp"

InspectionMission::InspectionMission() : Node("inspection") {
  std::unique_ptr<InspectionFunctions> inspection_object{new InspectionFunctions()};
  inspection_object->turning_period = declare_parameter<double>("turning_period", 4.0);
  inspection_object->finish_time = declare_parameter<double>("finish_time", 26.0);
  inspection_object->wheel_radius = declare_parameter<double>("wheel_radius", 0.254);
  inspection_object->max_angle = declare_parameter<double>("max_angle", 0.52359877559); // 30 degrees in rad
  inspection_object->start_and_stop = declare_parameter<bool>("start_and_stop", false);
  declare_parameter<double>("ebs_test_ideal_velocity", 2.0);
  declare_parameter<double>("ebs_test_gain", 0.25);
  declare_parameter<double>("inspection_ideal_velocity", 1.0);
  declare_parameter<double>("inspection_gain", 0.25);


  // creates publisher that should yield torque/acceleration/...
  control_command_publisher =
      this->create_publisher<fs_msgs::msg::ControlCommand>("/control_command", 10);

  // creates publisher for the flag
  finish_publisher = this->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);

  // get mission
  mission_signal = this->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&InspectionMission::mission_decider, this, std::placeholders::_1));

  rpm_subscription = this->create_subscription<fs_msgs::msg::WheelStates>(
      "/wheel_states", 10,
      std::bind(&InspectionMission::inspection_script, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Inspection node has been started.");
}

void InspectionMission::mission_decider(fs_msgs::msg::GoSignal mission_signal) {
  initial_time = std::chrono::system_clock::now();
  RCLCPP_INFO(this->get_logger(), "Mission received: %s", mission_signal.mission.c_str());
  if (mission == "inspection") {
    inspection_object->ideal_velocity = get_parameter("inspection_ideal_velocity").as_double();
    inspection_object->gain = get_parameter("inspection_gain").as_double();
  } else if (mission == "inspection_test_EBS") {
    inspection_object->ideal_velocity = get_parameter("ebs_test_ideal_velocity").as_double();
    inspection_object->gain = get_parameter("ebs_test_gain").as_double();
  } else {
    RCLCPP_INFO(this->get_logger(), "Invalid mission for inspection: %s", mission_signal.mission.c_str());
  }
  mission = mission_signal.mission;
}

void InspectionMission::publish_controls(double torque, double steering) {
  fs_msgs::msg::ControlCommand control_command;
  if (torque > 0) {
    control_command.throttle = torque;
    control_command.brake = 0;
  } else {
    control_command.throttle = 0;
    control_command.brake = -1.0 * torque;
  }
  control_command.steering = steering;
  control_command_publisher->publish(control_command);
}

void InspectionMission::inspection_script(fs_msgs::msg::WheelStates current_rpm) {
  // initialization
  auto current_time = std::chrono::system_clock::now();
  auto elapsed_time = (current_time - initial_time).count();
  float average_rpm = (current_rpm.rl_rpm + current_rpm.rr_rpm) / 2.0;
  double current_velocity = inspection_object->rpm_to_velocity(average_rpm);

  // calculate steering
  double calculated_steering = mission == "inspection" ?
  inspection_object->calculate_steering(elapsed_time / pow(10.0, 9)) : 0;

  //if the time is over, the car should be stopped
  if (elapsed_time >= (inspection_object->finish_time) * pow(10, 9)) inspection_object->current_goal_velocity = 0;

  // calculate torque and convert to control command
  double calculated_torque = inspection_object->calculate_throttle(current_velocity);

  // publish suitable message
  if (elapsed_time < (inspection_object->finish_time) * pow(10, 9) || std::abs(current_velocity) > 0.1) {
    RCLCPP_DEBUG(this->get_logger(), "Publishing control command. Steering: %f; Torque: %f",
                 calculated_steering, calculated_torque);
    publish_controls(calculated_torque, calculated_steering);
  } else {
    fs_msgs::msg::FinishedSignal finish;
    finish.placeholder = true;
    finish_publisher->publish(finish);
    if (mission == "ebs_test") {
      // transition to emergency
    }
  }

  // update ideal velocity if necessary
  inspection_object->redefine_goal_velocity(current_velocity);
}