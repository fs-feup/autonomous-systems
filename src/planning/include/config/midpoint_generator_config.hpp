#ifndef SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Midpoint Generator class.
 */
struct MidpointGeneratorConfig {
  /**
   * @brief Minimum distance between cones.
   */
  double minimum_cone_distance_;

  /**
   * @brief Maximum distance between cones.
   */
  double maximum_cone_distance_;

  /**
   * @brief Sliding window radius for midpoint generation.
   */
  double sliding_window_radius_;

  /**
   * @brief Default constructor.
   */
  MidpointGeneratorConfig() 
      : minimum_cone_distance_(2.0),
        maximum_cone_distance_(10.0),
        sliding_window_radius_(20.0) {}

  /**
   * @brief Parameterized constructor.
   * @param min_distance Minimum distance between cones
   * @param max_distance Maximum distance between cones
   * @param window_radius Sliding window radius
   */
  MidpointGeneratorConfig(double min_distance, double max_distance, double window_radius)
      : minimum_cone_distance_(min_distance),
        maximum_cone_distance_(max_distance),
        sliding_window_radius_(window_radius) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_