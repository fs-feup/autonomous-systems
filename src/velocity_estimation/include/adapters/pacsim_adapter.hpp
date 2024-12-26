#pragma once

#include "adapters/adapter.hpp"
#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "pacsim/msg/wheels.hpp"
#include "sensor_msgs/msg/imu.hpp"
class VENode;

class PacsimAdapter : public Adapter {
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr cog_imu_sub_;
  rclcpp::Subscription<pacsim::msg::Wheels>::SharedPtr wheel_speeds_sub_;

public:
  explicit PacsimAdapter(const VEParameters& parameters);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void WheelSpeedsCallback(const pacsim::msg::Wheels::SharedPtr msg);
};