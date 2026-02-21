#pragma once

#include "sensors/sensor_data.hpp"

class ImuData : public SensorData {
public:
  double yaw_rate_;
  double acceleration_x_;
  double acceleration_y_;

  ImuData() = default;
  ImuData(double yaw_rate_, double acceleration_x_, double acceleration_y_, rclcpp::Time timestamp);

  void measurement_model(State& state, const double dt) override;
};