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
   * @brief Use simulated velocities.
   */
  bool using_simulated_velocities_;

  /**
   * @brief Default constructor.
   */
  SimulationConfig()
      : publishing_visualization_msgs_(false),
        using_simulated_se_(false),
        using_simulated_velocities_(false) {}

  /**
   * @brief Parameterized constructor.
   */
  SimulationConfig(bool publishing_visualization_msgs, bool using_simulated_se,
                   bool using_simulated_velocities)
      : publishing_visualization_msgs_(publishing_visualization_msgs),
        using_simulated_se_(using_simulated_se),
        using_simulated_velocities_(using_simulated_velocities) {}
};

#endif  // SRC_PLANNING_INCLUDE_CONFIG_SIMULATION_CONFIG_HPP_