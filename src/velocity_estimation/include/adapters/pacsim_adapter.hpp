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
 * simulator. It then passes the data to the velocity estimator for processing.
 */
class PacsimAdapter : public VENode {
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub_;
  rclcpp::Subscription<pacsim::msg::Wheels>::SharedPtr wheel_speeds_sub_;
  rclcpp::Subscription<pacsim::msg::StampedScalar>::SharedPtr _steering_angle_subscription_;

public:
  explicit PacsimAdapter(const VEParameters& parameters);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void wss_callback(const pacsim::msg::Wheels::SharedPtr msg);
  void steering_angle_callback(const pacsim::msg::StampedScalar::SharedPtr msg);
};