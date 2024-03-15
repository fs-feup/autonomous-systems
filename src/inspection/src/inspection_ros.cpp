#include "include/inspection_ros.hpp"

InspectionMission::InspectionMission() : Node("inspection") {
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
      std::bind(&InspectionMission::inspection_general, this, std::placeholders::_1));
}

void InspectionMission::mission_decider(fs_msgs::msg::GoSignal mission_signal) {
  initial_time = std::chrono::system_clock::now();
  mission = mission_signal.mission;
  if (mission_signal.mission == "inspection_test_EBS") {
    inspection_object -> ideal_speed = 0;
  }
}

void InspectionMission::inspection_general(fs_msgs::msg::WheelStates current_rpm) {
  if (mission == "inspection") {
    inspection_script(current_rpm);
  } else if (mission == "inspection_test_EBS") {
    test_EBS(current_rpm);
  }
}

void InspectionMission::inspection_script(fs_msgs::msg::WheelStates current_rpm) {
  // initialization
  auto current_time = std::chrono::system_clock::now();
  auto elapsed_time =  (current_time - initial_time).count();
  auto control_command = fs_msgs::msg::ControlCommand();
  float average_rpm = (current_rpm.rl_rpm + current_rpm.rr_rpm)/2.0;
  double current_velocity = inspection_object -> rpm_to_speed(average_rpm);

  // calculate steering
  control_command.steering  = inspection_object -> calculate_steering(elapsed_time/pow(10.0, 9));

  // calculate torque
  double calculated_torque = inspection_object -> calculate_torque(current_velocity);
  if (calculated_torque > 0) {
    control_command.throttle = calculated_torque;
    control_command.brake = 0;
  } else {
    control_command.brake = -1.0*calculated_torque;
    control_command.throttle = 0;
  }

  // publish suitable message
  if (elapsed_time < (inspection_object->finish_time)*pow(10, 9)) {
    RCLCPP_INFO(this->get_logger(), "Publishing control command. Steering: %f; Torque: %f",
    control_command.steering, calculated_torque);
    control_command_publisher->publish(control_command);
  } else {
    fs_msgs::msg::FinishedSignal finish;
    finish.placeholder = true;
    finish_publisher->publish(finish);
  }

  // update ideal speed if necessary// calculate steering
  inspection_object -> redefine_ideal_speed(current_velocity);
}

void InspectionMission::test_EBS(fs_msgs::msg::WheelStates current_rpm) {
  auto current_time = std::chrono::system_clock::now();
  auto elapsed_time =  (current_time - initial_time).count();
  double average_rpm = (current_rpm.fl_rpm + current_rpm.fr_rpm +
  current_rpm.rl_rpm + current_rpm.rr_rpm)/4.0;
  double current_velocity = inspection_object -> rpm_to_speed(average_rpm);
  auto control_command = fs_msgs::msg::ControlCommand();

  // set steering to 0
  control_command.steering = 0;

  // calculate torque
  double calculated_torque = inspection_object -> calculate_torque(current_velocity);
  if (calculated_torque > 0) {
    control_command.throttle = calculated_torque;
    control_command.brake = 0;
  } else {
    control_command.throttle = 0;
    control_command.brake = -1.0*calculated_torque;
  }

  // publish suitable message
  if (elapsed_time < (inspection_object->finish_time)*pow(10, 9)) {
    RCLCPP_INFO(this->get_logger(), "Publishing control command. Steering: %f; Torque: %f",
    control_command.steering, calculated_torque);
    control_command_publisher->publish(control_command);
  } else {
    fs_msgs::msg::FinishedSignal finish;
    finish.placeholder = true;
    finish_publisher->publish(finish);
  }

  // update ideal speed if necessary
  inspection_object -> redefine_ideal_speed(current_velocity);
}
