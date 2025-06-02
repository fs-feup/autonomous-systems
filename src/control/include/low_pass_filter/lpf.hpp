#pragma once

/**
 * @brief Low Pass Filter class
 *
 * @details
 * This class implements a simple first-order low pass filter to be used in the Pure Pursuit controller.
 */
class LowPassFilter {
public:
  /**
   * @brief Construct a new Low Pass Filter object
   *
   * @param alpha Smoothing factor (0 < alpha < 1)
   * @param initial_value Initial filtered value
   */
  LowPassFilter(double alpha, double initial_value = 0.0);

  /**
   * @brief Apply the low pass filter to a new input
   *
   * @param input New input value
   * @return Filtered value
   */
  double filter(double input);

  /**
   * @brief Reset the filter to a specific value
   *
   * @param value Value to reset the filter to
   */
  void reset(double value = 0.0);

private:
  double alpha_;       /**< Smoothing factor */
  double prev_value_;  /**< Previous filtered value */
};