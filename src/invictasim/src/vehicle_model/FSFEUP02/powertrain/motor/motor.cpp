#include "vehicle_model/FSFEUP02/powertrain/motor/motor.hpp"

MotorModel::MotorModel(const MotorParameters& params) : params_(params), thermal_state_(0.0f) {
  // Calculate thermal capacity using I²t model
  // K = (I_peak² - I_cont²) × T_max_peak
  float i_peak_sq = params_.max_peak_current * params_.max_peak_current;
  float i_cont_sq = params_.max_continuous_current * params_.max_continuous_current;
  thermal_capacity_ = (i_peak_sq - i_cont_sq) * params_.peak_duration;
}

float MotorModel::getEfficiency(float torque, float rpm) const {
  // Handle efficiency map
  if (params_.efficiency_map.empty()) {
    return 0.95f;  // Default efficiency if no map provided
  }

  // Find closest RPM entry
  auto rpm_upper = params_.efficiency_map.lower_bound(rpm);

  if (rpm_upper == params_.efficiency_map.end()) {
    // RPM beyond map, use last entry
    rpm_upper = std::prev(params_.efficiency_map.end());
  }

  if (rpm_upper == params_.efficiency_map.begin()) {
    // RPM before map, use first entry
    // Just use this efficiency map
    return interpolateFromMap(rpm_upper->second, torque);
  }

  // Interpolate between two RPM values
  auto rpm_lower = std::prev(rpm_upper);

  float eff_lower = interpolateFromMap(rpm_lower->second, torque);
  float eff_upper = interpolateFromMap(rpm_upper->second, torque);

  float rpm1 = rpm_lower->first;
  float rpm2 = rpm_upper->first;

  // Linear interpolation
  float t = (rpm - rpm1) / (rpm2 - rpm1);
  return eff_lower + t * (eff_upper - eff_lower);
}

float MotorModel::getMaxTorqueAtRPM(float rpm) const {
  // Determine max torque based on thermal state
  float thermal_ratio = thermal_state_ / thermal_capacity_;

  // Interpolate between peak and continuous torque curves
  float max_torque_continuous = interpolateFromMap(params_.torque_speed_continuous, rpm);
  float max_torque_peak = interpolateFromMap(params_.torque_speed_peak, rpm);

  // As thermal state increases, available torque decreases from peak to continuous
  float max_torque = max_torque_peak - thermal_ratio * (max_torque_peak - max_torque_continuous);

  return std::max(0.0f, max_torque);
}

void MotorModel::updateState(float current_draw, float dt) {
  // I²t thermal model
  // Heat generation: H = (I² - I_cont²) * dt
  float i_cont_sq = params_.max_continuous_current * params_.max_continuous_current;
  float i_current_sq = current_draw * current_draw;

  float heat_generation = std::max(0.0f, i_current_sq - i_cont_sq) * dt;

  // Heat dissipation: exponential cooling towards zero
  float cooling_rate = thermal_state_ / params_.peak_duration;
  float heat_dissipation = cooling_rate * dt;

  // Update thermal state
  thermal_state_ += heat_generation - heat_dissipation;
  thermal_state_ = std::max(0.0f, std::min(thermal_capacity_, thermal_state_));
}

void MotorModel::reset() { thermal_state_ = 0.0f; }

float MotorModel::interpolateFromMap(const std::map<float, float>& map, float key) const {
  if (map.empty()) {
    return 0.0f;
  }

  // Find bounds
  auto upper = map.lower_bound(key);

  // Key is beyond the map range
  if (upper == map.end()) {
    return map.rbegin()->second;  // Return last value
  }

  // Key is before the map range
  if (upper == map.begin()) {
    return upper->second;  // Return first value
  }

  // Interpolate between two points
  auto lower = std::prev(upper);

  float x1 = lower->first;
  float y1 = lower->second;
  float x2 = upper->first;
  float y2 = upper->second;

  // Linear interpolation
  float t = (key - x1) / (x2 - x1);
  return y1 + t * (y2 - y1);
}
