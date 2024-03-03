#include "inspection/inspection.hpp"

InspectionMission::InspectionMission() : Node("inspection") {
  // creates publisher that should yield torque/acceleration/...
  control_command_publisher =
  this->create_publisher<fs_msgs::msg::ControlCommand>("control_topic", 10);

  // creates publisher for the flag
  flag_publisher = this->create_publisher<std_msgs::msg::String>("inspection_state", 10);

  // get velocity data from state estimation
  rpm_subscription = this->create_subscription<fs_msgs::msg::WheelStates>(
      "velocity_estimation", 10,
      std::bind(&InspectionMission::publish_torque_steer, this, std::placeholders::_1));
}

void InspectionMission::publish_torque_steer(fs_msgs::msg::WheelStates current_rpm) {
  auto current_time = std::chrono::system_clock::now();
  auto elapsed_time =  (current_time - initial_time).count();
  if (elapsed_time < 26000000000) {
    auto control_command = fs_msgs::msg::ControlCommand();
    float average_rpm = (current_rpm.rl_rpm + current_rpm.rr_rpm)/2.0;
    double calculated_torque = calculate_torque(rpm_to_speed(average_rpm));
    control_command.steering  = calculate_steering(elapsed_time/pow(10.0, 9));

    if (calculated_torque > 0) {
      control_command.throttle = calculated_torque;
    } else {
      control_command.brake = -1.0*calculated_torque;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing control command. Steering: %f; Torque: %f",
    control_command.steering, calculated_torque);

    control_command_publisher->publish(control_command);
  } else {
    auto flag = std_msgs::msg::String();
    flag.data = "AS Finished";
    flag_publisher->publish(flag);
    rclcpp::shutdown();
  }
}

double InspectionMission::calculate_steering(double time) {
    return sin((time*44.0)/(7.0*turning_period))*max_angle;
}

double InspectionMission::calculate_torque(double velocity) {
    double error = ideal_speed - velocity;
    return gain*error;
}

double InspectionMission::rpm_to_speed(double rpm) {
    double perimeter = 44.0*wheel_radius/7.0;
    return rpm*perimeter/60.0;
}
