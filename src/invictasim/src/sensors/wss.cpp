#include "sensors/wss.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "config/yaml_helpers.hpp"

namespace {
constexpr double kRadSToRpm = 60.0 / (2.0 * M_PI);
}  // namespace

WSS::WSS(const std::string& config_path) {
  using invictasim::config::load_optional;

  const YAML::Node wss = YAML::LoadFile(config_path)["wss_sensor"];

  // Indices follow WheelPosition: FR, FL, RR, RL.
  const char* wheel_keys[] = {"fr", "fl", "rr", "rl"};

  for (int i = 0; i < 4; ++i) {
    const YAML::Node wheel_node = wss[wheel_keys[i]];
    if (!wheel_node) {
      continue;
    }

    load_optional(wheel_node["noise"], "std_dev", wheel_params_[i].noise_std_dev);
    load_optional(wheel_node["outliers"], "probability", wheel_params_[i].outlier_probability);
    load_optional(wheel_node["outliers"], "impact_factor", wheel_params_[i].outlier_impact_factor);
    load_optional(wheel_node, "dropout_probability", wheel_params_[i].dropout_probability);
    load_optional(wheel_node, "speed_cutoff", wheel_params_[i].speed_cutoff);
    load_optional(wheel_node, "quantization_bits", wheel_params_[i].quantization_bits);

    wheel_params_[i].quantization_bits =
        std::clamp(wheel_params_[i].quantization_bits, 0, kMaxQuantizationBits);
  }
}

common_lib::structures::Wheels WSS::simulate_wheel_speeds(
    const common_lib::structures::Wheels& wheel_speeds) {
  for (int i = 0; i < 4; ++i) {
    wheel_dropout_[i] = false;
  }

  return common_lib::structures::Wheels(
      apply_wss_error(wheel_speeds.front_left * kRadSToRpm, WheelPosition::FRONT_LEFT),
      apply_wss_error(wheel_speeds.front_right * kRadSToRpm, WheelPosition::FRONT_RIGHT),
      apply_wss_error(wheel_speeds.rear_left * kRadSToRpm, WheelPosition::REAR_LEFT),
      apply_wss_error(wheel_speeds.rear_right * kRadSToRpm, WheelPosition::REAR_RIGHT));
}

double WSS::apply_wss_error(double rotational_speed_rpm, WheelPosition wheel) {
  const WheelParameters& params = wheel_params_[static_cast<int>(wheel)];

  // The cutoff is a property of the wheel, not of the reading: below it no teeth pass the pickup,
  // so the sensor reports exactly zero rather than noise about zero. Testing the noisy value
  // instead would let a parked car report its wheels turning.
  if (std::abs(rotational_speed_rpm) < params.speed_cutoff * kRadSToRpm) {
    return 0.0;
  }

  double measurement = rotational_speed_rpm + gaussian_noise(params.noise_std_dev * kRadSToRpm);
  measurement = apply_outlier(measurement, wheel);
  measurement = apply_dropout(measurement, wheel);

  if (params.quantization_bits > 0) {
    measurement = apply_quantization(measurement, wheel);
  }
  return measurement;
}

double WSS::apply_outlier(double value, WheelPosition wheel) {
  const WheelParameters& params = wheel_params_[static_cast<int>(wheel)];

  if (random_uniform() < params.outlier_probability) {
    const double sign = (random_uniform() < 0.5) ? -1.0 : 1.0;
    return value + sign * params.outlier_impact_factor * std::abs(value);
  }
  return value;
}

double WSS::apply_dropout(double value, WheelPosition wheel) {
  const int wheel_idx = static_cast<int>(wheel);
  if (random_uniform() < wheel_params_[wheel_idx].dropout_probability) {
    wheel_dropout_[wheel_idx] = true;
    return 0.0;
  }
  return value;
}

std::vector<bool> WSS::get_wheel_dropout_status() const {
  return std::vector<bool>(wheel_dropout_, wheel_dropout_ + 4);
}

double WSS::apply_quantization(double value, WheelPosition wheel) const {
  const WheelParameters& params = wheel_params_[static_cast<int>(wheel)];

  const long long levels = 1LL << params.quantization_bits;
  const double step_size = (2.0 * kQuantizationRangeRpm) / static_cast<double>(levels - 1);
  const double clamped = std::clamp(value, -kQuantizationRangeRpm, kQuantizationRangeRpm);

  return std::round((clamped + kQuantizationRangeRpm) / step_size) * step_size -
         kQuantizationRangeRpm;
}
