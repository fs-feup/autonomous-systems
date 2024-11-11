#ifndef SRC_PLANNING_INCLUDE_CONFIG_CONE_COLORING_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_CONE_COLORING_CONFIG_HPP_

/**
 * @brief struct for the configuration of the cone coloring algorithm.
 *
 */
struct ConeColoringConfig {
  /**
   * @brief angle gain between the last edge and the possible new edge
   *
   */
  double angle_weight_ = 11;

  /**
   * @brief distance gain to use for the distance cost
   *
   */
  double distance_weight_ = 8;

  /**
   * @brief gain to use for the cost
   *
   */
  double ncones_weight_ = 8.7;

  /**
   * @brief exponent for the distance
   *
   */
  double distance_exponent_ = 0.698;

  /**
   * @brief exponent for the angle
   *
   */
  double angle_exponent_ = 5.3;

  /**
   * @brief maximum cost to reach a cone
   *
   */
  double max_cost_ = 40;

  /**
   * @brief whether to memorize cones
   *
   */
  bool use_memory_ = true;
  ConeColoringConfig() = default;
  ConeColoringConfig(double angle_gain, double distance_gain, double ncones_gain, double exponent1,
                     double exponent2, double max_cost, bool use_memory)
      : angle_weight_(angle_gain),
        distance_weight_(distance_gain),
        ncones_weight_(ncones_gain),
        distance_exponent_(exponent1),
        angle_exponent_(exponent2),
        max_cost_(max_cost),
        use_memory_(use_memory) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_CONE_COLORING_CONFIG_HPP_