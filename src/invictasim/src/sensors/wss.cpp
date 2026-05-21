#include "sensors/wss.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <random>

WSS::WSS(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node wss = config["wss_sensor"];

  // Load white noise parameters
  YAML::Node noise = wss["noise"];
  noise_std_dev_ = noise["std_dev"].as<double>();

  // Load outlier parameters
  YAML::Node outliers = wss["outliers"];
  outlier_probability_ = outliers["probability"].as<double>();
  outlier_impact_factor_ = outliers["impact_factor"].as<double>();

  // Load dropout parameters
  dropout_probability_ = wss["dropout_probability"].as<double>();

  // Load speed cutoff
  speed_cutoff_ = wss["speed_cutoff"].as<double>();

  // Load quantization parameters
  if (wss["quantization_steps"]) {
    quantization_bits_ = wss["quantization_bits"].as<int>();
  } else {
    quantization_bits_ = 0;  // Disabled by default
  }
}

common_lib::structures::Wheels WSS::process_wheel_speeds(double wheel_speeds_fl,
                                                          double wheel_speeds_fr,
                                                          double wheel_speeds_rl,
                                                          double wheel_speeds_rr) {
  // Apply sensor errors to individual wheel speeds (in rad/s)
  double fl_noisy = apply_wss_error(wheel_speeds_fl);
  double fr_noisy = apply_wss_error(wheel_speeds_fr);
  double rl_noisy = apply_wss_error(wheel_speeds_rl);
  double rr_noisy = apply_wss_error(wheel_speeds_rr);

  // Convert from rad/s to rpm: rpm = rad_s * 60 / (2 * pi)
  constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
  
  double fl_rpm = fl_noisy * RAD_S_TO_RPM;
  double fr_rpm = fr_noisy * RAD_S_TO_RPM;
  double rl_rpm = rl_noisy * RAD_S_TO_RPM;
  double rr_rpm = rr_noisy * RAD_S_TO_RPM;
  
  // Apply quantization if enabled
  if (quantization_bits_ > 0) {
    fl_rpm = apply_quantization(fl_rpm);
    fr_rpm = apply_quantization(fr_rpm);
    rl_rpm = apply_quantization(rl_rpm);
    rr_rpm = apply_quantization(rr_rpm);
  }
  
  return common_lib::structures::Wheels(fl_rpm, fr_rpm, rl_rpm, rr_rpm);
}

double WSS::apply_wss_error(double rotational_speed) {
  // Apply speed cutoff
  double measurement = rotational_speed;
  if (std::abs(rotational_speed) < speed_cutoff_) {
    measurement = 0.0;
  }

  // Apply white Gaussian noise
  measurement = measurement + gaussian_noise(noise_std_dev_);

  // Apply outlier
  measurement = apply_outlier(measurement);

  // Apply dropout
  measurement = apply_dropout(measurement);

  return measurement;
}


double WSS::apply_outlier(double value) {
  // Check if outlier should occur
  if (random_uniform() < outlier_probability_) {
    // Generate outlier by scaling the current measurement
    // The sign is randomly chosen
    double sign = (random_uniform() < 0.5) ? -1.0 : 1.0;
    return value + (sign * outlier_impact_factor_ * std::abs(value));
  }
  return value;
}

double WSS::apply_dropout(double value) {
  // Check if dropout should occur
  if (random_uniform() < dropout_probability_) {
    return 0.0;
  }
  return value;
}

double WSS::apply_quantization(double value) {
  // Quantize the value based on the configured bit depth
  // Assumes range of [-4000, 4000] rpm (sufficient for typical wheel speeds)
  constexpr double MIN_RPM = -4000.0;
  constexpr double MAX_RPM = 4000.0;
  
  // Calculate the number of levels: 2^bits
  long long num_levels = 1LL << quantization_bits_;  // 2^quantization_bits_
  
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
