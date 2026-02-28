#ifndef SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_
#define SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_

/**
 * @brief Configuration parameters for the Simulation class.
 */
struct SimulationConfig {
  /**
   * @brief Flag to enable/disable publishing of visualization messages.
   */
  bool publishing_visualization_msgs_;

  /**
   * @brief Use simulated State Estimation.
   */
  bool using_simulated_se_;

  /**
   * @brief Default constructor.
   */
  SimulationConfig() : publishing_visualization_msgs_(false), using_simulated_se_(false) {}

  /**
   * @brief Parameterized constructor.
   */
  SimulationConfig(bool publishing_visualization_msgs, bool using_simulated_se)
      : publishing_visualization_msgs_(publishing_visualization_msgs),
        using_simulated_se_(using_simulated_se) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_