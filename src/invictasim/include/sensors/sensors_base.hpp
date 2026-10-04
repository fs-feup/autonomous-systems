#pragma once

#include <cmath>
#include <random>
#include <vector>

/**
 * @brief Base class for sensor implementations.
 *
 * Owns the random generator shared by the noise, outlier and dropout models.
 */
class Sensor {
public:
  virtual ~Sensor() = default;

  /**
   * @brief Draw zero-mean Gaussian noise.
   *
   * @param std_dev Standard deviation. Values at or below zero return 0.0, because
   *                std::normal_distribution requires a positive standard deviation.
   * @return A sample from the distribution, or 0.0 when noise is disabled.
   */
  double gaussian_noise(double std_dev);

  /**
   * @brief Draw a uniform sample in [0, 1].
   */
  double random_uniform();

protected:
  Sensor();
  std::mt19937 generator_;
};
