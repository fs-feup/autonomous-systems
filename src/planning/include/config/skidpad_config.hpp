#ifndef SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Skidpad class.
 *
 */
struct SkidpadConfig {
  /**
   * @brief Minimum number of cones required.
   * 
   */
  int minimum_cones_ = 10;

  /**
   * @brief Maximum distance for matching cone pairs during ICP alignment.
   *
   */
  double tolerance_ = 2.0;

  SkidpadConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_