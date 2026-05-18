#include "sensors/wss.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <random>

WSS::WSS(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node wss = config["wss_sensor"];

  // Load quantization parameters
  quantization_step_ = wss["quantization_step"].as<double>();

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
}

double WSS::apply_wss_error(double rotational_speed) {
  // Step 1: Apply speed cutoff (below cutoff, output is zero)
  if (std::abs(rotational_speed) < speed_cutoff_) {
    rotational_speed = 0.0;
  }

  // Step 2: Apply white Gaussian noise
  double measurement = rotational_speed + gaussian_noise(noise_std_dev_);

  // Step 3: Apply quantization
  measurement = apply_quantization(measurement);

  // Step 4: Apply outlier
  measurement = apply_outlier(measurement);

  // Step 5: Apply dropout
  measurement = apply_dropout(measurement);

  return measurement;
}

double WSS::apply_quantization(double value) {
  if (quantization_step_ <= 0.0) {
    return value;
  }
  // Round to nearest quantization step
  return std::round(value / quantization_step_) * quantization_step_;
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

double WSS::random_uniform() {
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(Sensor::generator_);
}
