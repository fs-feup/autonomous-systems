#pragma once

#include "adapters/adapter.hpp"
#include "common_lib/sensor_data/imu.hpp"
#include "sensor_msgs/msg/imu.hpp"
class VENode;

class PacsimAdapter : public Adapter {
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr cog_imu_sub_;

public:
  explicit PacsimAdapter(const VEParameters& parameters);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
};