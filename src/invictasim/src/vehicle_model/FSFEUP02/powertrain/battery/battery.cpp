#include "vehicle_model/FSFEUP02/powertrain/battery/battery_model.hpp"

BatteryModel::BatteryModel(const BatteryParameters& params)
    : params_(params), soc_(1.0f), energy_consumed_(0.0f), thermal_state_(0.0f) {
  // Calculate thermal capacity using I²t model
  // K = (I_peak² - I_cont²) × T_max_peak
  float i_peak_sq = params_.max_peak_discharge_current * params_.max_peak_discharge_current;
  float i_cont_sq =
      params_.max_continuous_discharge_current * params_.max_continuous_discharge_current;
  thermal_capacity_ = (i_peak_sq - i_cont_sq) * params_.peak_duration;
}

float BatteryModel::getVoltage(float current_draw) const {
  // Terminal voltage = OCV - I*R
  float ocv = getOpenCircuitVoltage();
  return ocv - (current_draw * params_.internal_resistance);
}

float BatteryModel::getOpenCircuitVoltage() const { return interpolateVoltage(soc_); }

void BatteryModel::updateState(float current_draw, float dt) {
  // Update SOC based on current draw
  // Ah consumed = I * dt / 3600 (converting seconds to hours)
  float ah_consumed = (current_draw * dt) / 3600.0f;
  float soc_change = ah_consumed / params_.capacity_ah;
  soc_ -= soc_change;

  // Clamp SOC to valid range
  soc_ = std::max(0.0f, std::min(1.0f, soc_));

  // Update energy consumed
  float voltage = getVoltage(current_draw);
  float power = voltage * current_draw;      // Watts
  float energy_wh = (power * dt) / 3600.0f;  // Wh
  energy_consumed_ += energy_wh;

  // Update thermal state
  updateThermalState(current_draw, dt);
}

float BatteryModel::getAvailablePower() const {
  float allowed_current = getAllowedDischargeCurrent();
  float voltage = getVoltage(allowed_current);
  return voltage * allowed_current;
}

void BatteryModel::updateThermalState(float current_draw, float dt) {
  // I²t thermal model
  // Heat generation: H = (I² - I_cont²) * dt
  float i_cont_sq =
      params_.max_continuous_discharge_current * params_.max_continuous_discharge_current;
  float i_current_sq = current_draw * current_draw;

  float heat_generation = std::max(0.0f, i_current_sq - i_cont_sq) * dt;

  // Heat dissipation: exponential cooling towards zero
  // Assume time constant equal to peak_duration for simplicity
  float cooling_rate = thermal_state_ / params_.peak_duration;
  float heat_dissipation = cooling_rate * dt;

  // Update thermal state
  thermal_state_ += heat_generation - heat_dissipation;
  thermal_state_ = std::max(0.0f, std::min(thermal_capacity_, thermal_state_));
}

float BatteryModel::getAllowedDischargeCurrent() const {
  // Calculate allowed current based on thermal state
  // When thermal_state is at capacity, only continuous current is allowed
  // When thermal_state is at zero, peak current is allowed

  float thermal_ratio = thermal_state_ / thermal_capacity_;

  // Linear interpolation between peak and continuous
  float allowed_current = params_.max_peak_discharge_current -
                          thermal_ratio * (params_.max_peak_discharge_current -
                                           params_.max_continuous_discharge_current);

  // Also consider SOC limitations
  if (soc_ < params_.min_soc) {
    allowed_current = 0.0f;
  }

  return allowed_current;
}

bool BatteryModel::isSafe() const {
  float voltage = getVoltage(0.0f);  // Check OCV

  if (voltage < params_.min_voltage || voltage > params_.max_voltage) {
    return false;
  }

  if (soc_ < params_.min_soc) {
    return false;
  }

  return true;
}

void BatteryModel::reset(float initial_soc) {
  soc_ = std::max(0.0f, std::min(1.0f, initial_soc));
  energy_consumed_ = 0.0f;
  thermal_state_ = 0.0f;
}

float BatteryModel::interpolateVoltage(float soc) const {
  return interpolateFromMap(params_.soc_voltage_map, soc);
}

float BatteryModel::calculateHeatGeneration(float current_draw) const {
  // I²R losses
  return current_draw * current_draw * params_.internal_resistance;
}

float BatteryModel::interpolateFromMap(const std::map<float, float>& map, float key) const {
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

void BatteryModel::calculateCurrentForPower(float electrical_power_req, float& current,
                                            float& electrical_power) const {
  // Equation 6.15: I = (VOCV - sqrt(VOCV² - 4*Rint*Pelec)) / (2*Rint)
  float ocv = getOpenCircuitVoltage();
  float r_int = params_.internal_resistance;

  // Initialize outputs
  current = 0.0f;
  electrical_power = 0.0f;

  // Check if power is achievable
  float discriminant = ocv * ocv - 4.0f * r_int * electrical_power_req;

  if (discriminant < 0.0f || r_int < 1e-6f) {
    // Power not achievable with current battery state
    return;
  }

  // Calculate current using equation 6.15
  float sqrt_discriminant = std::sqrt(discriminant);
  float calculated_current = (ocv - sqrt_discriminant) / (2.0f * r_int);

  // Step 4: Electrical Limit Enforcement
  // Check against battery current limits (Imax)
  float max_battery_current = getAllowedDischargeCurrent();

  // Also check minimum terminal voltage (Vmin)
  float terminal_voltage = ocv - calculated_current * r_int;

  if (calculated_current > max_battery_current || terminal_voltage < params_.min_voltage) {
    // Battery limits exceeded, clamp current
    calculated_current = std::min(calculated_current, max_battery_current);

    // Recalculate terminal voltage with clamped current
    terminal_voltage = ocv - calculated_current * r_int;

    // Ensure terminal voltage doesn't go below minimum
    if (terminal_voltage < params_.min_voltage) {
      calculated_current = (ocv - params_.min_voltage) / r_int;
      terminal_voltage = params_.min_voltage;
    }
  }

  // Populate output parameters
  current = std::max(0.0f, calculated_current);
  electrical_power = terminal_voltage * current;
}
