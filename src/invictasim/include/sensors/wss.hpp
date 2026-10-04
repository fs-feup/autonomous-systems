#pragma once

#include <string>
#include <vector>

#include "common_lib/structures/wheels.hpp"
#include "sensors/sensors_base.hpp"

/**
 * @brief Wheel speed sensor (WSS) simulator.
 *
 * Simulates, per wheel: white Gaussian noise, random outliers, signal dropout, ADC quantisation,
 * and a speed cutoff below which the sensor reads exactly zero.
 */
class WSS : public Sensor {
public:
  /**
   * @brief Wheel positions, in the order the configuration file lists them.
   */
  enum class WheelPosition {
    FRONT_RIGHT = 0,  ///< Front right wheel
    FRONT_LEFT = 1,   ///< Front left wheel
    REAR_RIGHT = 2,   ///< Rear right wheel
    REAR_LEFT = 3     ///< Rear left wheel
  };

  /**
   * @brief Per-wheel sensor error parameters.
   */
  struct WheelParameters {
    double noise_std_dev = 0.0;          ///< Gaussian noise standard deviation (rad/s)
    double outlier_probability = 0.0;    ///< Probability of an outlier per sample (0 to 1)
    double outlier_impact_factor = 0.0;  ///< Outlier magnitude as a fraction of the reading
    double dropout_probability = 0.0;    ///< Probability of a dropout per sample (0 to 1)
    double speed_cutoff = 0.0;           ///< Reading below this magnitude reports zero (rad/s)
    int quantization_bits = 0;           ///< ADC resolution in bits (0 disables quantisation)
  };

  /**
   * @brief Construct a new WSS object and load configuration.
   * @param config_path Path to the wss.yaml configuration file.
   */
  explicit WSS(const std::string& config_path);

  /**
   * @brief Corrupt all four ground-truth wheel speeds and convert them to rpm.
   *
   * @param wheel_speeds True wheel angular speeds (rad/s).
   * @return Simulated wheel speeds in rpm.
   */
  common_lib::structures::Wheels simulate_wheel_speeds(
      const common_lib::structures::Wheels& wheel_speeds);

  /**
   * @brief Which wheels dropped out on the most recent call to simulate_wheel_speeds().
   * @return Four flags, indexed by WheelPosition.
   */
  std::vector<bool> get_wheel_dropout_status() const;

private:
  /// Full-scale range assumed by the ADC model, in rpm.
  static constexpr double kQuantizationRangeRpm = 4000.0;
  /// Largest shift the quantisation model accepts, to keep 1 << bits well defined.
  static constexpr int kMaxQuantizationBits = 24;

  WheelParameters wheel_params_[4];                       ///< Parameters per WheelPosition
  bool wheel_dropout_[4] = {false, false, false, false};  ///< Dropout flags per WheelPosition

  /**
   * @brief Apply the error model to one wheel, in rpm.
   *
   * Below the speed cutoff the sensor reports exactly zero, because no teeth pass the pickup.
   * Above it, noise, outliers, dropout and finally quantisation are applied.
   *
   * @param rotational_speed_rpm True wheel speed (rpm).
   * @param wheel Which wheel's parameters to use.
   * @return The corrupted reading (rpm).
   */
  double apply_wss_error(double rotational_speed_rpm, WheelPosition wheel);

  /**
   * @brief Inject an outlier with the configured probability.
   */
  double apply_outlier(double value, WheelPosition wheel);

  /**
   * @brief Zero the reading with the configured dropout probability, recording that it happened.
   */
  double apply_dropout(double value, WheelPosition wheel);

  /**
   * @brief Quantise a reading to the configured ADC resolution.
   */
  double apply_quantization(double value, WheelPosition wheel) const;
};
