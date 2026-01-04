#ifndef SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Midpoint Generator class.
 */
struct MidpointGeneratorConfig {
  /**
   * @brief Minimum distance between cones.
   */
  double minimum_cone_distance_ = 10.0;

  /**
   * @brief Maximum distance between cones.
   */
  double maximum_cone_distance_ = 2.0;

  /**
   * @brief Sliding window radius for midpoint generation.
   */
  double sliding_window_radius_ = 20.0;

  MidpointGeneratorConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_