#pragma once

#include <vector>
#include <string>
#include <memory>
#include "config/parameters.hpp"
#include "common_lib/structures/control_command.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

class SolverInterface {
protected:
    std::shared_ptr<ControlParameters> control_params_;
public:
    SolverInterface(const ControlParameters& control_params)
        : control_params_(std::make_shared<ControlParameters>(control_params)) {}

    virtual ~SolverInterface() {}

    /**
     * @brief Set the state, from state estimation. Includes pose and velocity estimation
     * 
     * @param x0 Current state vector
     */
    virtual void set_state(const custom_interfaces::msg::VehicleStateVector& state) = 0;


    /**
     * @brief Set the path from path planning
     * 
     * @param x0 Current state vector
     */
    virtual void set_path(const custom_interfaces::msg::PathPointArray& path) = 0;

    /**
     * @brief Set the previously sent command, for solvers that constrain command-to-command changes.
     *
     * Solvers that do not need this information can ignore it.
     */
    virtual void set_previous_control_command(
        const common_lib::structures::ControlCommand&) {}

    /**
     * @brief Solve the optimization problem
     * 
     * @param solver_status Optional pointer to an integer to store the solver status code (0 for success, 1 for benign failure such as infeasibility, negative for solver errors or divergences)
     * 
     * @return the computed control command for the current state
     */
    virtual common_lib::structures::ControlCommand solve(int* solver_status = nullptr) = 0;

    /**
     * @brief Get the full predicted control solution (for the whole horizon)
     * 
     */
    virtual std::vector<common_lib::structures::ControlCommand> get_full_solution() = 0;
    
    /**
     * @brief Get the predicted state for each stage of the horizon
     * 
     * @return std::vector<custom_interfaces::msg::VehicleStateVector> vector of states, one for each stage in the horizon
     */
    virtual std::vector<custom_interfaces::msg::VehicleStateVector> get_full_horizon() = 0;

    /**
     * @brief Publish any relevant solver data for visualization or debugging purposes.
     * 
     * @param node 
     * @param publisher_map 
     */
    virtual void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) = 0;
};
