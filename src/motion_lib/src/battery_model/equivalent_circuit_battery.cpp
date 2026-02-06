#include "equivalent_circuit_battery.hpp"

EquivalentCircuitBattery::EquivalentCircuitBattery(
    const common_lib::car_parameters::CarParameters& car_parameters)
    : BatteryModel(car_parameters), soc_(1.0f), thermal_state_(0.0f), current_(0.0f) {
  // Calculate thermal capacity using I²t model
  // K = (I_peak² - I_cont²) × T_max_peak
  float i_peak_sq = car_parameters_->battery_parameters->max_peak_discharge_current *
                    car_parameters_->battery_parameters->max_peak_discharge_current;
  float i_cont_sq = car_parameters_->battery_parameters->max_continuous_discharge_current *
                    car_parameters_->battery_parameters->max_continuous_discharge_current;
  this->thermal_capacity_ =
      (i_peak_sq - i_cont_sq) * car_parameters_->battery_parameters->peak_duration;
}

std::tuple<float, float> EquivalentCircuitBattery::calculateCurrentForPower(
    float electrical_power_req) const {
  // I = (VOCV - sqrt(VOCV² - 4*Rint*Pelec)) / (2*Rint)
  float ocv = getOpenCircuitVoltage();

  // Initialize outputs
  float current = 0.0f;
  float electrical_power = 0.0f;

  // Check if power is achievable
  float discriminant = ocv * ocv - 4.0f * car_parameters_->battery_parameters->internal_resistance *
                                       electrical_power_req;

  if (discriminant < 0.0f) {
    // Power not achievable with current battery state
    return std::make_tuple(current, electrical_power);
  }

  float sqrt_discriminant = std::sqrt(discriminant);
  float calculated_current =
      (ocv - sqrt_discriminant) / (2.0f * car_parameters_->battery_parameters->internal_resistance);

  // Check against battery current limits
  float max_battery_current = getAllowedDischargeCurrent();

  // Also check minimum terminal voltage (Vmin)
  float terminal_voltage =
      ocv - calculated_current * car_parameters_->battery_parameters->internal_resistance;

  if (calculated_current > max_battery_current ||
      terminal_voltage < car_parameters_->battery_parameters->min_voltage) {
    // Battery limits exceeded, clamp current
    calculated_current = std::min(calculated_current, max_battery_current);

    // Recalculate terminal voltage with clamped current
    terminal_voltage =
        ocv - calculated_current * car_parameters_->battery_parameters->internal_resistance;

    // Ensure terminal voltage doesn't go below minimum
    if (terminal_voltage < car_parameters_->battery_parameters->min_voltage) {
      calculated_current = (ocv - car_parameters_->battery_parameters->min_voltage) /
                           car_parameters_->battery_parameters->internal_resistance;
      terminal_voltage = car_parameters_->battery_parameters->min_voltage;
    }
  }

  // Populate output parameters
  current = std::max(0.0f, calculated_current);
  electrical_power = terminal_voltage * current;
  return std::make_tuple(current, electrical_power);
}

float EquivalentCircuitBattery::getVoltage() const {
  // Terminal voltage = OCV - I*R
  float ocv = getOpenCircuitVoltage();
  return ocv - (this->current_ * car_parameters_->battery_parameters->internal_resistance);
}

float EquivalentCircuitBattery::getVoltage(float current_draw) const {
  // Terminal voltage = OCV - I*R
  float ocv = getOpenCircuitVoltage();
  return ocv - (current_draw * car_parameters_->battery_parameters->internal_resistance);
}

float EquivalentCircuitBattery::getOpenCircuitVoltage() const {
  return interpolateVoltage(this->soc_);
}

void EquivalentCircuitBattery::updateState(float current_draw, float dt) {
  // Update SOC based on current draw
  // Ah consumed = I * dt / 3600 (converting seconds to hours)
  float ah_consumed = (current_draw * dt) / 3600.0f;
  float soc_change = ah_consumed / car_parameters_->battery_parameters->capacity_ah;
  soc_ -= soc_change;

  // Clamp SOC to valid range
  soc_ = std::max(0.0f, std::min(1.0f, soc_));

  // heat generation: H = (I² - I_cont²) * dt
  float i_cont_sq = car_parameters_->battery_parameters->max_continuous_discharge_current *
                    car_parameters_->battery_parameters->max_continuous_discharge_current;
  float i_current_sq = current_draw * current_draw;

  float heat_generation = std::max(0.0f, i_current_sq - i_cont_sq) * dt;

  // Heat dissipation: exponential cooling towards zero
  // Assume time constant equal to peak_duration for simplicity
  float cooling_rate = thermal_state_ / car_parameters_->battery_parameters->peak_duration;
  float heat_dissipation = cooling_rate * dt;

  // Update thermal state
  thermal_state_ += heat_generation - heat_dissipation;
  thermal_state_ = std::max(0.0f, std::min(thermal_capacity_, thermal_state_));
}

void EquivalentCircuitBattery::reset() {
  soc_ = 1.0f;
  current_ = 0.0f;
  thermal_state_ = 0.0f;
}

float EquivalentCircuitBattery::getAllowedDischargeCurrent() const {
  // Calculate allowed current based on thermal state
  // When thermal_state is at capacity, only continuous current is allowed
  // When thermal_state is at zero, peak current is allowed

  float thermal_ratio = thermal_state_ / thermal_capacity_;

  // Linear interpolation between peak and continuous
  float allowed_current =
      car_parameters_->battery_parameters->max_peak_discharge_current -
      thermal_ratio * (car_parameters_->battery_parameters->max_peak_discharge_current -
                       car_parameters_->battery_parameters->max_continuous_discharge_current);

  // Also consider SOC limitations
  if (soc_ < car_parameters_->battery_parameters->min_soc) {
    allowed_current = 0.0f;
  }

  return allowed_current;
}

float EquivalentCircuitBattery::interpolateFromMap(const std::map<float, float>& map,
                                                   float key) const {
  if (map.empty()) {
    throw std::runtime_error("Cannot interpolate from empty map");
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
