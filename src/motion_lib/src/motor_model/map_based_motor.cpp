#include "motion_lib/motor_model/map_based_motor.hpp"

MapBasedMotor::MapBasedMotor(const common_lib::car_parameters::CarParameters& car_parameters)
    : MotorModel(car_parameters), current_(0.0f), thermal_state_(0.0f) {
  // K = (I_peak² - I_cont²) × T_max_peak
  double i_peak_sq = car_parameters_->motor_parameters->max_peak_current *
                     car_parameters_->motor_parameters->max_peak_current;
  double i_cont_sq = car_parameters_->motor_parameters->max_continuous_current *
                     car_parameters_->motor_parameters->max_continuous_current;
  this->thermal_capacity_ =
      (i_peak_sq - i_cont_sq) * car_parameters_->motor_parameters->peak_duration;
}

double MapBasedMotor::get_efficiency(double torque, double rpm) const {
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

  double eff_lower = interpolate_from_map(rpm_lower->second, torque);
  double eff_upper = interpolate_from_map(rpm_upper->second, torque);

  double rpm1 = rpm_lower->first;
  double rpm2 = rpm_upper->first;

  // Linear interpolation
  double t = (rpm - rpm1) / (rpm2 - rpm1);
  return eff_lower + t * (eff_upper - eff_lower);
}

double MapBasedMotor::get_max_torque_at_rpm(double rpm) const {
  double thermal_ratio = thermal_state_ / thermal_capacity_;

  // Calculate limits based on current state
  // Torque: 230Nm -> 102Nm | Power: 120kW -> 60kW | RPM: 6500 -> 5500
  double current_max_t =
      car_parameters_->motor_parameters->max_peak_torque -
      (thermal_ratio * (car_parameters_->motor_parameters->max_peak_torque -
                        car_parameters_->motor_parameters->max_continous_torque));

  double current_max_p =
      car_parameters_->motor_parameters->max_peak_power -
      (thermal_ratio * (car_parameters_->motor_parameters->max_peak_power -
                        car_parameters_->motor_parameters->max_continuous_power));

  double current_max_rpm =
      car_parameters_->motor_parameters->max_peak_rpm -
      (thermal_ratio * (car_parameters_->motor_parameters->max_peak_rpm -
                        car_parameters_->motor_parameters->max_continuous_rpm));

  if (rpm >= current_max_rpm) {
    return 0.0f;  // Should not happen due to 1/x decay, but just in case
  }

  if (rpm < 0.1) {
    return current_max_t;
  }

  double omega = (rpm * 2.0 * static_cast<double>(M_PI)) / 60.0;

  // 1/x decay logic to the limits as we approach max RPM
  double proximity_to_limit = 1.0 - (rpm / current_max_rpm);
  double power_limited_torque = (current_max_p / omega) * proximity_to_limit;

  return std::min(current_max_t, power_limited_torque);
}

void MapBasedMotor::update_state(double current_draw, double torque, double dt) {
  // I²t thermal model
  // Heat generation: H = (I² - I_cont²) * dt
  double i_cont_sq = car_parameters_->motor_parameters->max_continuous_current *
                     car_parameters_->motor_parameters->max_continuous_current;
  double i_current_sq = current_draw * current_draw;

  double heat_generation = std::max(0.0, i_current_sq - i_cont_sq) * dt;

  // Heat dissipation: exponential cooling towards zero
  double cooling_rate = this->thermal_state_ / car_parameters_->motor_parameters->peak_duration;
  double heat_dissipation = cooling_rate * dt;

  // Update thermal state
  this->thermal_state_ += heat_generation - heat_dissipation;
  this->thermal_state_ = std::max(0.0, std::min(this->thermal_capacity_, this->thermal_state_));
  this->current_ = current_draw;
  this->torque_ = torque;
}

void MapBasedMotor::reset() {
  this->thermal_state_ = 0.0;
  this->current_ = 0.0;
  this->torque_ = 0.0;
}

double MapBasedMotor::get_torque() const { return this->torque_; }

double MapBasedMotor::get_current() const { return this->current_; }

double MapBasedMotor::get_thermal_state() const { return this->thermal_state_; }

double MapBasedMotor::get_thermal_capacity() const { return this->thermal_capacity_; }

double MapBasedMotor::interpolate_from_map(const std::map<double, double>& map, double key) const {
  if (map.empty()) {
    return 0.0;
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

  double x1 = lower->first;
  double y1 = lower->second;
  double x2 = upper->first;
  double y2 = upper->second;

  // Linear interpolation
  double t = (key - x1) / (x2 - x1);
  return y1 + t * (y2 - y1);
}
