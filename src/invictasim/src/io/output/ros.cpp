#include "io/output/ros.hpp"

#include <cmath>

RosOutputAdapter::RosOutputAdapter(const std::shared_ptr<InvictaSim>& simulator)
    : Node("invictasim_output", rclcpp::NodeOptions().use_global_arguments(false)),
      InvictaSimOutputAdapter(simulator) {
  motor_torque_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/motor/torque", 10);
  battery_current_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/current", 10);
  battery_voltage_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/voltage", 10);
  battery_soc_pub_ =
      this->create_publisher<std_msgs::msg::Float64>("/invictasim/battery/state_of_charge", 10);
  vehicle_state_pub_ = this->create_publisher<custom_interfaces::msg::VehicleStateVector>(
      "/invictasim/vehicle_state", 10);
}

void RosOutputAdapter::publish_outputs(const std::shared_ptr<VehicleModel>& vehicle_model,
                                       double steering_angle) {
  custom_interfaces::msg::VehicleStateVector vehicle_state_msg;
  vehicle_state_msg.velocity_x = vehicle_model->get_velocity_x();
  vehicle_state_msg.velocity_y = vehicle_model->get_velocity_y();
  vehicle_state_msg.yaw_rate = vehicle_model->get_yaw_rate();
  vehicle_state_msg.acceleration_x = vehicle_model->get_acceleration_x();
  vehicle_state_msg.acceleration_y = vehicle_model->get_acceleration_y();
  vehicle_state_msg.steering_angle = steering_angle;
  auto wheels_speed = vehicle_model->get_wheels_speed();
  vehicle_state_msg.fl_rpm = wheels_speed.front_left * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.fr_rpm = wheels_speed.front_right * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rl_rpm = wheels_speed.rear_left * 60.0 / (2.0 * M_PI);
  vehicle_state_msg.rr_rpm = wheels_speed.rear_right * 60.0 / (2.0 * M_PI);
  vehicle_state_pub_->publish(vehicle_state_msg);

  std_msgs::msg::Float64 msg;
  msg.data = vehicle_model->get_motor_torque();
  motor_torque_pub_->publish(msg);
  msg.data = vehicle_model->get_battery_current();
  battery_current_pub_->publish(msg);
  msg.data = vehicle_model->get_battery_voltage();
  battery_voltage_pub_->publish(msg);
  msg.data = vehicle_model->get_battery_soc();
  battery_soc_pub_->publish(msg);
}
