#include "sensors/sensors_base.hpp"

Sensor::Sensor() : generator_(rd_()) {}

double Sensor::gaussian_noise(double std_dev) {
  std::normal_distribution<double> distribution(0.0, std_dev);
  return distribution(generator_);
}
