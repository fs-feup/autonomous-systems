#pragma once

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "node/node.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "sensor_msgs/msg/imu.hpp"

/**
 * @brief Adapter class for the pacsim simulator.
 *
 * This class subscribes to the IMU, wheel speeds, and steering angle topics published by the pacsim
 * simulator. It then passes the data to the state estimator for processing.
 */
class PacsimAdapter : public SENode {
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub_;
  rclcpp::Subscription<pacsim::msg::Wheels>::SharedPtr _wheel_speeds_sub_;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr _steering_angle_sub_;
  rclcpp::Subscription<pacsim::msg::Wheels>::SharedPtr _throttle_command_sub_;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr _steering_command_sub_;

  common_lib::structures::ControlCommand _latest_control_command;
  bool received_steering_command_ = false;
  bool received_throttle_command_ = false;

public:
  explicit PacsimAdapter(const SEParameters& parameters);

  /**
   * @brief callback that gets called when the throttle command is received
   */
  void throttle_command_callback(const pacsim::msg::Wheels::SharedPtr msg);

  /**
   * @brief callback that gets called when the steering command is received
   */
  void steering_command_callback(const pacsim::msg::StampedScalar::SharedPtr msg);

  /**
   * @brief callback that gets called when the IMU data is received
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief callback that gets called when the wheel speeds are received
   */
  void wss_callback(const pacsim::msg::Wheels::SharedPtr msg);

  /**
   * @brief callback that gets called when the steering angle is received
   */
  void steering_angle_callback(const pacsim::msg::StampedScalar::SharedPtr msg);
};