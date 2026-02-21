#pragma once

#include "sensors/sensor_data.hpp"

class WSSData : public SensorData {
public:
  double fl_wheel_rpm_;
  double fr_wheel_rpm_;
  double rl_wheel_rpm_;
  double rr_wheel_rpm_;

  WSSData() = default;
  WSSData(double fl_wheel_rpm_, double fr_wheel_rpm_, double rl_wheel_rpm_, double rr_wheel_rpm_,
          rclcpp::Time timestamp);

  void measurement_model(State& state, const double dt) override;
};