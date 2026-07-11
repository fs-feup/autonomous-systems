#include "sensors/steering_sensor.hpp"

#include <yaml-cpp/yaml.h>

SteeringSensor::SteeringSensor(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node sensor = config["steering_angle_sensor"];

  noise_std_dev_ = sensor["noise_std_dev"].as<double>();
  scale_factor_ = sensor["scale_factor"].as<double>();
  bias_ = sensor["bias"].as<double>();
  steering_speed_scale_ = sensor["steering_speed_scale"].as<double>();
  quantization_step_ = sensor["quantization_step"].as<double>();
}

double SteeringSensor::apply_sas_error(double steering_angle_raw) {
  double steering_with_bias = Sensor::apply_bias(steering_angle_raw, bias_);
  return Sensor::quantize(steering_with_bias + gaussian_noise(noise_std_dev_),
                          quantization_step_);
}
