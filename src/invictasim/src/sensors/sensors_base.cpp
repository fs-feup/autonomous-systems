#include "sensors/sensors_base.hpp"

Sensor::Sensor() : generator_(std::random_device{}()) {}

double Sensor::gaussian_noise(double std_dev) {
  if (!(std_dev > 0.0)) {
    return 0.0;
  }
  std::normal_distribution<double> distribution(0.0, std_dev);
  return distribution(generator_);
}

double Sensor::random_uniform() {
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(generator_);
}
