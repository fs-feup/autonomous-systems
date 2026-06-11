#pragma once

#include <cmath>
#include <random>
#include <vector>

/**
 * @brief Abstract base class for sensor implementations
 *
 * Provides Gaussian noise implementation for sensor error modeling.
 */
class Sensor {
public:
  virtual ~Sensor() = default;

  /**
   * @brief Generate Gaussian noise
   *
   * @param std_dev Standard deviation of the Gaussian distribution
   * @return double Random value from the Gaussian distribution
   */
  double gaussian_noise(double std_dev);

protected:
  Sensor();
  std::random_device rd_;
  std::mt19937 generator_;
};