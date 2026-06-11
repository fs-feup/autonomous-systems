#include "sensors/wss.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <random>
#include <common_lib/config_load/config_load.hpp>

WSS::WSS(const std::string& config_path) {
  std::string wss_cfg = common_lib::config_load::get_config_yaml_path(
    "invictasim", "invictasim/sensors", "wss");
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node wss = config["wss_sensor"];

  // Wheel configuration keys in order: FR, FL, RR, RL
  const char* wheel_keys[] = {"fr", "fl", "rr", "rl"};

  // Load parameters for each wheel
  for (int i = 0; i < 4; ++i) {
    YAML::Node wheel_node = wss[wheel_keys[i]];

    // Load noise parameters
    YAML::Node noise = wheel_node["noise"];
    wheel_params_[i].noise_std_dev = noise["std_dev"].as<double>();

    // Load outlier parameters
    YAML::Node outliers = wheel_node["outliers"];
    wheel_params_[i].outlier_probability = outliers["probability"].as<double>();
    wheel_params_[i].outlier_impact_factor = outliers["impact_factor"].as<double>();

    // Load dropout parameters
    wheel_params_[i].dropout_probability = wheel_node["dropout_probability"].as<double>();

    // Load speed cutoff
    wheel_params_[i].speed_cutoff = wheel_node["speed_cutoff"].as<double>();

    // Load quantization parameters
    if (wheel_node["quantization_steps"]) {
      wheel_params_[i].quantization_steps = wheel_node["quantization_steps"].as<int>();
    } else {
      wheel_params_[i].quantization_steps = 0;  // Disabled by default
    }
  }
}

common_lib::structures::Wheels WSS::simulate_wheel_speeds(double wheel_speeds_fl,
                                                          double wheel_speeds_fr,
                                                          double wheel_speeds_rl,
                                                          double wheel_speeds_rr) {
  // Reset dropout tracking for this cycle
  for (int i = 0; i < 4; ++i) {
    wheel_dropout_[i] = false;
  }

  // Apply sensor errors to individual wheel speeds (in rad/s)
  double fl_noisy = apply_wss_error(wheel_speeds_fl, WheelPosition::FRONT_LEFT);
  double fr_noisy = apply_wss_error(wheel_speeds_fr, WheelPosition::FRONT_RIGHT);
  double rl_noisy = apply_wss_error(wheel_speeds_rl, WheelPosition::REAR_LEFT);
  double rr_noisy = apply_wss_error(wheel_speeds_rr, WheelPosition::REAR_RIGHT);

  // Convert from rad/s to rpm: rpm = rad_s * 60 / (2 * pi)
  constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
  
  double fl_rpm_simulated = fl_noisy * RAD_S_TO_RPM;
  double fr_rpm_simulated = fr_noisy * RAD_S_TO_RPM;
  double rl_rpm_simulated = rl_noisy * RAD_S_TO_RPM;
  double rr_rpm_simulated = rr_noisy * RAD_S_TO_RPM;
  
  // Apply quantization if enabled for each wheel
  if (wheel_params_[static_cast<int>(WheelPosition::FRONT_LEFT)].quantization_steps > 0) {
    fl_rpm_simulated = apply_quantization(fl_rpm_simulated, WheelPosition::FRONT_LEFT);
  }
  if (wheel_params_[static_cast<int>(WheelPosition::FRONT_RIGHT)].quantization_steps > 0) {
    fr_rpm_simulated = apply_quantization(fr_rpm_simulated, WheelPosition::FRONT_RIGHT);
  }
  if (wheel_params_[static_cast<int>(WheelPosition::REAR_LEFT)].quantization_steps > 0) {
    rl_rpm_simulated = apply_quantization(rl_rpm_simulated, WheelPosition::REAR_LEFT);
  }
  if (wheel_params_[static_cast<int>(WheelPosition::REAR_RIGHT)].quantization_steps > 0) {
    rr_rpm_simulated = apply_quantization(rr_rpm_simulated, WheelPosition::REAR_RIGHT);
  }
  
  return common_lib::structures::Wheels(fl_rpm_simulated, fr_rpm_simulated, rl_rpm_simulated, rr_rpm_simulated);
}

double WSS::apply_wss_error(double rotational_speed, WheelPosition wheel) {
  int wheel_idx = static_cast<int>(wheel);
  const WheelParameters& params = wheel_params_[wheel_idx];

  // Apply speed cutoff
  double measurement = rotational_speed;
  if (std::abs(rotational_speed) < params.speed_cutoff) {
    measurement = 0.0;
  }

  // Apply white Gaussian noise
  measurement = measurement + gaussian_noise(params.noise_std_dev);

  // Apply outlier
  measurement = apply_outlier(measurement, wheel);

  // Apply dropout
  measurement = get_dropout_info(measurement, wheel);

  return measurement;
}

double WSS::apply_outlier(double value, WheelPosition wheel) {
  int wheel_idx = static_cast<int>(wheel);
  const WheelParameters& params = wheel_params_[wheel_idx];

  // Check if outlier should occur
  if (random_uniform() < params.outlier_probability) {
    // Generate outlier by scaling the current measurement
    // The sign is randomly chosen
    double sign = (random_uniform() < 0.5) ? -1.0 : 1.0;
    return value + (sign * params.outlier_impact_factor * std::abs(value));
  }
  return value;
}

double WSS::get_dropout_info(double value, WheelPosition wheel) {
  int wheel_idx = static_cast<int>(wheel);
  const WheelParameters& params = wheel_params_[wheel_idx];

  // Check if dropout should occur
  if (random_uniform() < params.dropout_probability) {
    wheel_dropout_[wheel_idx] = true;
    return 0.0;
  }
  return value;
}

std::vector<bool> WSS::get_wheel_dropout_status() const {
  return std::vector<bool>(wheel_dropout_, wheel_dropout_ + 4);
}

double WSS::apply_quantization(double value, WheelPosition wheel) {
  int wheel_idx = static_cast<int>(wheel);
  const WheelParameters& params = wheel_params_[wheel_idx];

  // Quantize the value based on the configured bit depth
  // Assumes range of [-4000, 4000] rpm
  constexpr double MIN_RPM = -4000.0;
  constexpr double MAX_RPM = 4000.0;
  
  // Calculate the number of levels: 2^bits
  long long num_levels = 1LL << params.quantization_steps;  // 2^quantization_steps
  
  // Calculate step size: (max - min) / (levels - 1)
  double step_size = (MAX_RPM - MIN_RPM) / (num_levels - 1);
  
  // Clamp value to range
  double clamped = std::max(MIN_RPM, std::min(MAX_RPM, value));
  
  // Quantize: find nearest level
  double quantized = std::round((clamped - MIN_RPM) / step_size) * step_size + MIN_RPM;
  
  return quantized;
}

double WSS::random_uniform() {
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(Sensor::generator_);
}
