#ifndef SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct SkidpadConfig {
  int skidpad_minimum_cones_ = 10.0;
  double skidpad_tolerance_ = 1.0;
  SkidpadConfig() = default;
  SkidpadConfig(int skidpad_minimum_cones, double skidpad_tolerance)
      : skidpad_minimum_cones_(skidpad_minimum_cones),
      skidpad_tolerance_(skidpad_tolerance) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SKIDPAD_CONFIG_HPP_