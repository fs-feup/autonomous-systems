#ifndef SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct SkidpadConfig {
  int minimum_cones_ = 10.0;
  double tolerance_ = 1.0;

  SkidpadConfig() = default;
  SkidpadConfig(int minimum_cones, double tolerance)
      : minimum_cones_(minimum_cones),
      tolerance_(tolerance) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_