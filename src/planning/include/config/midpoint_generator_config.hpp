#ifndef SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_

/**
 * @brief Configuration for Delaunay triangulation and midpoint generation
 */
struct MidpointGeneratorConfig {
  // Distance constraints for valid cone pairs
  double minimum_cone_distance_ = 10.0;
  double maximum_cone_distance_ = 2.0;
  
  // Sliding window filtering
  bool use_sliding_window_ = true;
  double sliding_window_radius_ = 20.0;

  MidpointGeneratorConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_MIDPOINT_GENERATOR_CONFIG_HPP_