#include "limited_slip_differential.hpp"

// Still not implemented, just a simple distribution for finishing the refactor of configs and
// structure
common_lib::structures::Wheels LimitedSlipDifferential::calculateTorqueDistribution(
    float input_torque, const common_lib::structures::Wheels& wheel_speeds) const {
  // Placeholder implementation: Distribute torque equally to all wheels
  // In a real implementation, this would consider wheel slip and other factors
  float torque_per_wheel = input_torque / 2.0f;
  return common_lib::structures::Wheels(0, 0, torque_per_wheel, torque_per_wheel);
}