#pragma once

#include <vector>
#include <string>
#include <memory>
#include "config/parameters.hpp"
#include "common_lib/structures/control_command.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/vehicle_state.hpp"


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
    virtual void set_state(const std::vector<double>& x0) = 0;


    /**
     * @brief Set the path from path planning
     * 
     * @param x0 Current state vector
     */
    virtual void set_path(const std::vector<double>& x0) = 0;

    /**
     * @brief Solve the optimization problem
     * 
     * @return the computed control command for the current state
     */
    virtual common_lib::structures::ControlCommand solve() = 0;

    /**
     * @brief Get the full predicted control solution (for the whole horizon)
     * 
     */
    virtual std::vector<common_lib::structures::ControlCommand> get_full_solution() = 0;
    
    /**
     * @brief Get the predicted state at a given stage in the horizon
     * 
     * @return std::vector<double> 
     */
    virtual std::vector<std::pair<common_lib::structures::Pose, common_lib::structures::VehicleState>> get_full_horizon() = 0;
};
