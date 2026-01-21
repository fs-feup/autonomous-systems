#ifndef SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Skidpad class.
 */
struct SkidpadConfig {
  /**
   * @brief Minimum number of cones required.
   */
  int minimum_cones_;

  /**
   * @brief Maximum distance for matching cone pairs during ICP alignment.
   */
  double tolerance_;

  /**
   * @brief Default constructor with sensible defaults.
   */
  SkidpadConfig() : minimum_cones_(10), tolerance_(2.0) {}

  /**
   * @brief Parameterized constructor.
   */
  SkidpadConfig(int minimum_cones, double tolerance)
      : minimum_cones_(minimum_cones), tolerance_(tolerance) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_