#include "sensors/sensors_base.hpp"

// Static member initialization
std::random_device Sensor::rd_;
std::mt19937 Sensor::generator_(Sensor::rd_());

double Sensor::gaussian_noise(double std_dev) {
  std::normal_distribution<double> distribution(0.0, std_dev);
  return distribution(generator_);
}

double Sensor::apply_bias(double value, double bias) const { return value + bias; }

double Sensor::quantize(double value, double resolution) const {
  if (resolution <= 0.0) {
    return value;
  }

  return std::round(value / resolution) * resolution;
}
