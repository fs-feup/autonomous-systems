#include "sensors/drivetrain.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

Drivetrain::Drivetrain(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);

  const YAML::Node steering = config["steering_sensor"];
  steering_noise_std_dev_ = steering["noise_std_dev"].as<double>();
  steering_scale_factor_ = steering["scale_factor"].as<double>();
  steering_bias_ = steering["bias"].as<double>();
  steering_resolution_ = steering["resolution"].as<double>();

  const YAML::Node resolver = config["resolver_sensor"];
  resolver_noise_std_dev_ = resolver["noise_std_dev"].as<double>();
  resolver_scale_factor_ = resolver["scale_factor"].as<double>();
  resolver_bias_ = resolver["bias"].as<double>();
}

double Drivetrain::apply_steering_error(double steering_angle) {
  double measurement = steering_angle + steering_bias_;
  measurement += steering_scale_factor_ * steering_angle;
  measurement += gaussian_noise(steering_noise_std_dev_);

  if (steering_resolution_ > 0.0) {
    measurement = std::round(measurement / steering_resolution_) * steering_resolution_;
  }

  return measurement;
}

double Drivetrain::apply_resolver_error(double motor_rpm) {
  double measurement = motor_rpm + resolver_bias_;
  measurement += resolver_scale_factor_ * motor_rpm;
  measurement += gaussian_noise(resolver_noise_std_dev_);
  return measurement;
}
