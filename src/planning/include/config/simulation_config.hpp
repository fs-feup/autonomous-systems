#ifndef SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct SimulationConfig {
  bool publishing_visualization_msgs_ = false;
  bool using_simulated_se_ = false;
  SimulationConfig() = default;
  SimulationConfig(bool publishing_visualization_msg, bool using_simulated_se)
      : publishing_visualization_msgs_(publishing_visualization_msg),
        using_simulated_se_(using_simulated_se) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_