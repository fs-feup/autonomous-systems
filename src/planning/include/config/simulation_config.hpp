#ifndef SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_

/**
 * @brief struct for the configuration of the smoothing algorithm
 *
 */
struct SimulationConfig {
  bool publishing_visualization_msgs = false;
  bool using_simulated_se = false;
  SimulationConfig() = default;
  SimulationConfig(bool publishing_visualization_msg, bool using_simulated_se)
      : publishing_visualization_msgs(publishing_visualization_msg),
        using_simulated_se(using_simulated_se) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_