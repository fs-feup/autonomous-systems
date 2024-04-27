#include "include/inspection_ros.hpp"

#include "exceptions/invalid_mission_exception.hpp"

#include "message_filters/message_traits.h"

#include "std_msgs/msg/string.hpp"

#include "message_filters/time_synchronizer.h"

#include "message_filters/sync_policies/approximate_time.h"


InspectionMission::InspectionMission() : Node("inspection") {
  double turning_period, finish_time, wheel_radius, max_angle;
  bool start_and_stop;

  turning_period = declare_parameter<double>("turning_period", turning_period);
  finish_time = declare_parameter<double>("finish_time", finish_time);
  wheel_radius = declare_parameter<double>("wheel_radius", wheel_radius);
  max_angle = declare_parameter<double>("max_angle", max_angle);
  start_and_stop = declare_parameter<bool>("start_and_stop", start_and_stop);
  declare_parameter<double>("ebs_test_ideal_velocity", rclcpp::PARAMETER_DOUBLE);
  declare_parameter<double>("ebs_test_gain", rclcpp::PARAMETER_DOUBLE);
  declare_parameter<double>("inspection_ideal_velocity", rclcpp::PARAMETER_DOUBLE);
  declare_parameter<double>("inspection_gain", rclcpp::PARAMETER_DOUBLE);

  this->inspection_object =
      new InspectionFunctions(max_angle, turning_period, wheel_radius, finish_time, start_and_stop);

  // creates publisher that should yield torque/acceleration/...
  control_command_publisher =
      this->create_publisher<fs_msgs::msg::ControlCommand>("/as_msgs/controls", 10);

  // creates publisher for the flag
  finish_publisher = this->create_publisher<fs_msgs::msg::FinishedSignal>("/as_msgs/mission_finished", 10);

  // get mission
  mission_signal = this->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&InspectionMission::mission_decider, this, std::placeholders::_1));
  
  rlRPM_subscription.subscribe(this, "rlRPM");
  rrRPM_subscription.subscribe(this, "rrRPM");

  // WSS Synchronization
  const WSSPolicy policy(10);
  sync_ = std::make_shared<message_filters::Synchronizer<WSSPolicy>>(policy, rlRPM_subscription, rrRPM_subscription);
  sync_->registerCallback(&InspectionMission::inspection_script, this);

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
    throw new InvalidMissionException(mission);
  }
  mission = mission_signal.mission;
}

void InspectionMission::inspection_script(custom_interfaces::msg::WheelRPM current_rlRPM, custom_interfaces::msg::WheelRPM current_rrRPM) {
  // initialization
  auto current_time = std::chrono::system_clock::now();
  auto elapsed_time = (current_time - initial_time).count();
  auto control_command = fs_msgs::msg::ControlCommand();
  float average_rpm = (current_rlRPM.rl_rpm + current_rrRPM.rr_rpm) / 2.0;
  double current_velocity = inspection_object->rpm_to_velocity(average_rpm);

  // calculate steering
  control_command.steering =
      mission == "inspection" ? inspection_object->calculate_steering(elapsed_time / pow(10.0, 9))
                              : 0;

  // calculate torque
  double calculated_torque = inspection_object->calculate_throttle(current_velocity);
  if (calculated_torque > 0) {
    control_command.throttle = calculated_torque;
    control_command.brake = 0;
  } else {
    control_command.brake = -1.0 * calculated_torque;
    control_command.throttle = 0;
  }

  // publish suitable message
  if (elapsed_time < (inspection_object->finish_time) * pow(10, 9)) {
    RCLCPP_DEBUG(this->get_logger(), "Publishing control command. Steering: %f; Torque: %f",
                 control_command.steering, calculated_torque);
    control_command_publisher->publish(control_command);
  } else {
    fs_msgs::msg::FinishedSignal finish;
    finish.placeholder = true;
    finish_publisher->publish(finish);
  }

  // update ideal velocity if necessary
  inspection_object->redefine_goal_velocity(current_velocity);
}