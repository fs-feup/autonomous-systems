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
  double angle_weight = 11;

  /**
   * @brief distance gain to use for the distance cost
   *
   */
  double distance_weight = 8;

  /**
   * @brief gain to use for the cost
   *
   */
  double ncones_weight = 8.7;

  /**
   * @brief exponent for the distance
   *
   */
  double distance_exponent = 0.698;

  /**
   * @brief exponent for the angle
   *
   */
  double angle_exponent = 5.3;

  /**
   * @brief maximum cost to reach a cone
   *
   */
  double max_cost = 40;
  ConeColoringConfig() = default;
  ConeColoringConfig(double angle_gain, double distance_gain, double ncones_gain, double exponent1,
                     double exponent2, double max_cost)
      : angle_weight(angle_gain),
        distance_weight(distance_gain),
        ncones_weight(ncones_gain),
        distance_exponent(exponent1),
        angle_exponent(exponent2),
        max_cost(max_cost) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_CONE_COLORING_CONFIG_HPP_