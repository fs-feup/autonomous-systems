#pragma once

#include <cmath>
#include <random>
#include <vector>

/**
 * @brief Abstract base class for sensor implementations
 *
 * Provides Gaussian noise implementation, bias, and quantization for sensor error modeling.
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

  /**
   * @brief Apply a constant bias offset to a sensor reading.
   *
   * @param value Raw sensor value
   * @param bias Bias offset to add
   * @return double Value with bias applied
   */
  double apply_bias(double value, double bias) const;

  /**
   * @brief Quantize a value to a given resolution.
   *
   * @param value Raw sensor value
   * @param resolution Quantization step size
   * @return double Quantized value
   */
  double quantize(double value, double resolution) const;

protected:
  Sensor();
  std::random_device rd_;
  std::mt19937 generator_;
};