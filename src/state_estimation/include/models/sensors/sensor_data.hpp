#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "utils/parameters.hpp"
#include "utils/state_define.hpp"

class SensorData {
public:
  const rclcpp::Time timestamp;
  int dimensionality;
  int id;
  virtual void h(State& state, const double dt) = 0;
};