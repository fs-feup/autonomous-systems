#pragma once

#include <string>

#include "common_lib/structures/wheels.hpp"
#include "sensors/sensors_base.hpp"

/**
 * @brief Wheel Speed Sensor (WSS) simulator
 *
 * Simulates realistic wheel speed sensor characteristics including:
 * - White Gaussian noise on rotational speed measurements
 * - Random outliers with configurable probability and magnitude
 * - Signal dropout events
 * - Speed cutoff (zero output below minimum threshold)
 * - Returns wheel speeds in rpm
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
   * @brief Process all four wheel speeds and return them in rpm
   *
   * Applies sensor errors to all wheel speeds and converts from rad/s to rpm.
   *
   * @param wheel_speeds_fl Front left wheel speed (rad/s)
   * @param wheel_speeds_fr Front right wheel speed (rad/s)
   * @param wheel_speeds_rl Rear left wheel speed (rad/s)
   * @param wheel_speeds_rr Rear right wheel speed (rad/s)
   * @return common_lib::structures::Wheels Wheel speeds in rpm
   */
  common_lib::structures::Wheels process_wheel_speeds(double wheel_speeds_fl,
                                                      double wheel_speeds_fr,
                                                      double wheel_speeds_rl,
                                                      double wheel_speeds_rr);

  /**
   * @brief Apply realistic sensor errors to rotational speed measurement
   *
   * Applies the following error sources in sequence:
   * 1. Speed cutoff check
   * 2. White noise
   * 3. Outliers
   * 4. Dropout
   *
   * @param rotational_speed Ground truth rotational speed (rad/s)
   * @return double The corrupted measurement (rad/s)
   */
  double apply_wss_error(double rotational_speed);

private:

  // White noise parameters
  double noise_std_dev_;  ///< Gaussian noise standard deviation (rad/s)

  // Outlier parameters
  double outlier_probability_;  ///< Probability of outlier (0.0 to 1.0)
  double outlier_impact_factor_;  ///< Scaling factor for outlier magnitude

  // Dropout parameters
  double dropout_probability_;  ///< Probability of dropout (0.0 to 1.0)

  // Speed cutoff
  double speed_cutoff_;  ///< Minimum speed threshold (rad/s)

  // Quantization
  int quantization_bits_;  ///< Number of bits for ADC quantization (0 to disable)

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
   * @brief Apply quantization (ADC resolution) to a value
   *
   * Quantizes the input value based on the configured bit depth.
   * Assumes a range of [-4000, 4000] rpm for 12-bit quantization.
   *
   * @param value Value in rpm to quantize
   * @return double Quantized value in rpm
   */
  double apply_quantization(double value);

  /**
   * @brief Generate random number between 0.0 and 1.0
   *
   * @return double Random value in [0.0, 1.0]
   */
  double random_uniform();
};
