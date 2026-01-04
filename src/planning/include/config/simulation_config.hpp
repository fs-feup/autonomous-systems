#ifndef SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Simulation class.
 *
 */
struct SimulationConfig {
  /**
   * @brief Flag to enable/disable publishing of visualization messages.
   */
  bool publishing_visualization_msgs_ = false;

  /**
   * @brief Use simulated State Estimation
   */
  bool using_simulated_se_ = false;

  SimulationConfig() = default;
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_