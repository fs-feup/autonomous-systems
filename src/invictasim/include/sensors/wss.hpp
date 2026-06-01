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
 * 
 * Each wheel can be individually configured with its own error parameters.
 */
class WSS : public Sensor {
public:
  /**
   * @brief Enumeration for wheel positions
   */
  enum class WheelPosition {
    FRONT_RIGHT = 0,  ///< Front right wheel
    FRONT_LEFT = 1,   ///< Front left wheel
    REAR_RIGHT = 2,   ///< Rear right wheel
    REAR_LEFT = 3     ///< Rear left wheel
  };

  /**
   * @brief Structure to hold per-wheel sensor parameters
   */
  struct WheelParameters {
    double noise_std_dev;          ///< Gaussian noise standard deviation (rad/s)
    double outlier_probability;    ///< Probability of outlier (0.0 to 1.0)
    double outlier_impact_factor;  ///< Scaling factor for outlier magnitude
    double dropout_probability;    ///< Probability of dropout (0.0 to 1.0)
    double speed_cutoff;           ///< Minimum speed threshold (rad/s)
    int quantization_steps;        ///< Number of steps for ADC quantization (0 to disable)
  };

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
  common_lib::structures::Wheels simulate_wheel_speeds(double wheel_speeds_fl,
                                                      double wheel_speeds_fr,
                                                      double wheel_speeds_rl,
                                                      double wheel_speeds_rr);

  /**
   * @brief Apply realistic sensor errors to rotational speed measurement for a specific wheel
   *
   * Applies the following error sources in sequence:
   * 1. Speed cutoff check
   * 2. White noise
   * 3. Outliers
   * 4. Dropout
   *
   * @param rotational_speed Ground truth rotational speed (rad/s)
   * @param wheel Wheel position identifier
   * @return double The corrupted measurement (rad/s)
   */
  double apply_wss_error(double rotational_speed, WheelPosition wheel);

  /**
   * @brief Get the dropout status for all wheels
   *
   * @return std::vector<bool> Vector indicating which wheels had dropout (true = dropout occurred)
   */
  std::vector<bool> get_wheel_dropout_status() const;

private:

  // Per-wheel parameters
  WheelParameters wheel_params_[4];  ///< Parameters for each wheel (FR, FL, RR, RL)
  bool wheel_dropout_[4] = {false, false, false, false};  ///< Track if dropout occurred for each wheel

  /**
   * @brief Apply random outlier with configurable probability
   *
   * @param value Current measurement value
   * @param wheel Wheel position identifier
   * @return double Value with potential outlier injection
   */
  double apply_outlier(double value, WheelPosition wheel);

  /**
   * @brief Check for signal dropout and update dropout tracking
   *
   * @param value Current measurement value
   * @param wheel Wheel position identifier
   * @return double Zero if dropout occurs, otherwise the input value
   */
  double get_dropout_info(double value, WheelPosition wheel);

  /**
   * @brief Apply quantization (ADC resolution) to a value
   *
   * Quantizes the input value based on the configured bit depth.
   * Assumes a range of [-4000, 4000] rpm for 12-bit quantization.
   *
   * @param value Value in rpm to quantize
   * @param wheel Wheel position identifier
   * @return double Quantized value in rpm
   */
  double apply_quantization(double value, WheelPosition wheel);

  /**
   * @brief Generate random number between 0.0 and 1.0
   *
   * @return double Random value in [0.0, 1.0]
   */
  double random_uniform();
};
