#include "sensors/sensors_base.hpp"

// Static member initialization
std::random_device Sensor::rd_;
std::mt19937 Sensor::generator_(Sensor::rd_());

double Sensor::gaussian_noise(double std_dev) {
  std::normal_distribution<double> distribution(0.0, std_dev);
  return distribution(generator_);
}
