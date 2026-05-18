#pragma once

#include <string>

#include "sensors/sensors_base.hpp"

/**
 * @brief Wheel Speed Sensor (WSS) simulator
 *
 * Simulates realistic wheel speed sensor characteristics including:
 * - White Gaussian noise on rotational speed measurements
 * - Quantization error (discrete resolution)
 * - Random outliers with configurable probability and magnitude
 * - Signal dropout events
 * - Speed cutoff (zero output below minimum threshold)
 */
class WSS : public Sensor {
public:
  /**
   * @brief Construct a new WSS object and load configuration
   *
   * @param config_path Path to the wss.yaml configuration file
   */
  explicit WSS(const std::string& config_path);

  /**
   * @brief Apply realistic sensor errors to rotational speed measurement
   *
   * Applies the following error sources in sequence:
   * 1. Speed cutoff check
   * 2. White noise
   * 3. Quantization
   * 4. Outliers
   * 5. Dropout
   *
   * @param rotational_speed Ground truth rotational speed (rad/s)
   * @return double The corrupted measurement (rad/s)
   */
  double apply_wss_error(double rotational_speed);

private:
  // Quantization parameters
  double quantization_step_;  ///< Quantization resolution (rad/s)

  // White noise parameters
  double noise_std_dev_;  ///< Gaussian noise standard deviation (rad/s)

  // Outlier parameters
  double outlier_probability_;  ///< Probability of outlier (0.0 to 1.0)
  double outlier_impact_factor_;  ///< Scaling factor for outlier magnitude

  // Dropout parameters
  double dropout_probability_;  ///< Probability of dropout (0.0 to 1.0)

  // Speed cutoff
  double speed_cutoff_;  ///< Minimum speed threshold (rad/s)

  /**
   * @brief Apply quantization to the measurement
   *
   * @param value Raw measurement value
   * @return double Quantized value
   */
  double apply_quantization(double value);

  /**
   * @brief Apply random outlier with configurable probability
   *
   * @param value Current measurement value
   * @return double Value with potential outlier injection
   */
  double apply_outlier(double value);

  /**
   * @brief Check for signal dropout
   *
   * @param value Current measurement value
   * @return double Zero if dropout occurs, otherwise the input value
   */
  double apply_dropout(double value);

  /**
   * @brief Generate random number between 0.0 and 1.0
   *
   * @return double Random value in [0.0, 1.0]
   */
  double random_uniform();
};
