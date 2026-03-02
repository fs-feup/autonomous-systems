#include "motion_lib/motor_model/map_based_motor.hpp"

MapBasedMotor::MapBasedMotor(const common_lib::car_parameters::CarParameters& car_parameters)
    : MotorModel(car_parameters), current_(0.0f), thermal_state_(0.0f) {
  // K = (I_peak² - I_cont²) × T_max_peak
  float i_peak_sq = car_parameters_->motor_parameters->max_peak_current *
                    car_parameters_->motor_parameters->max_peak_current;
  float i_cont_sq = car_parameters_->motor_parameters->max_continuous_current *
                    car_parameters_->motor_parameters->max_continuous_current;
  this->thermal_capacity_ =
      (i_peak_sq - i_cont_sq) * car_parameters_->motor_parameters->peak_duration;
}

float MapBasedMotor::get_efficiency(float torque, float rpm) const {
  // Handle efficiency map
  if (car_parameters_->motor_parameters->efficiency_map.empty()) {
    return 0.95f;  // Default efficiency if no map provided
  }

  // Find closest RPM entry
  auto rpm_upper = car_parameters_->motor_parameters->efficiency_map.lower_bound(rpm);

  if (rpm_upper == car_parameters_->motor_parameters->efficiency_map.end()) {
    // RPM beyond map, use last entry
    rpm_upper = std::prev(car_parameters_->motor_parameters->efficiency_map.end());
  }

  if (rpm_upper == car_parameters_->motor_parameters->efficiency_map.begin()) {
    // RPM before map, use first entry
    // Just use this efficiency map
    return interpolate_from_map(rpm_upper->second, torque);
  }

  // Interpolate between two RPM values
  auto rpm_lower = std::prev(rpm_upper);

  float eff_lower = interpolate_from_map(rpm_lower->second, torque);
  float eff_upper = interpolate_from_map(rpm_upper->second, torque);

  float rpm1 = rpm_lower->first;
  float rpm2 = rpm_upper->first;

  // Linear interpolation
  float t = (rpm - rpm1) / (rpm2 - rpm1);
  return eff_lower + t * (eff_upper - eff_lower);
}

float MapBasedMotor::get_max_torque_at_rpm(float rpm) const {
  float thermal_ratio = thermal_state_ / thermal_capacity_;

  // Calculate limits based on current state
  // Torque: 230Nm -> 102Nm | Power: 120kW -> 60kW | RPM: 6500 -> 5500
  float current_max_t = car_parameters_->motor_parameters->max_peak_torque -
                        (thermal_ratio * (car_parameters_->motor_parameters->max_peak_torque -
                                          car_parameters_->motor_parameters->max_continous_torque));

  float current_max_p = car_parameters_->motor_parameters->max_peak_power -
                        (thermal_ratio * (car_parameters_->motor_parameters->max_peak_power -
                                          car_parameters_->motor_parameters->max_continuous_power));

  float current_max_rpm = car_parameters_->motor_parameters->max_peak_rpm -
                          (thermal_ratio * (car_parameters_->motor_parameters->max_peak_rpm -
                                            car_parameters_->motor_parameters->max_continuous_rpm));

  if (rpm < 0.1f) {
    return current_max_t;  // Max torque at standstill, no power limit
  }

  // Calculate max torque based on power limit
  float omega = (rpm * 2.0f * static_cast<float>(M_PI)) / 60.0f;
  float power_limited_torque = current_max_p / omega;
  return std::min(current_max_t, power_limited_torque);
}

void MapBasedMotor::update_state(float current_draw, float torque, float dt) {
  // I²t thermal model
  // Heat generation: H = (I² - I_cont²) * dt
  float i_cont_sq = car_parameters_->motor_parameters->max_continuous_current *
                    car_parameters_->motor_parameters->max_continuous_current;
  float i_current_sq = current_draw * current_draw;

  float heat_generation = std::max(0.0f, i_current_sq - i_cont_sq) * dt;

  // Heat dissipation: exponential cooling towards zero
  float cooling_rate = this->thermal_state_ / car_parameters_->motor_parameters->peak_duration;
  float heat_dissipation = cooling_rate * dt;

  // Update thermal state
  this->thermal_state_ += heat_generation - heat_dissipation;
  this->thermal_state_ = std::max(0.0f, std::min(this->thermal_capacity_, this->thermal_state_));
  this->torque_ = torque;
}

void MapBasedMotor::reset() {
  this->thermal_state_ = 0.0f;
  this->current_ = 0.0f;
  this->torque_ = 0.0f;
}

float MapBasedMotor::get_torque() const { return this->torque_; }

float MapBasedMotor::interpolate_from_map(const std::map<float, float>& map, float key) const {
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
