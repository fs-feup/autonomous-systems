#pragma once

#include "solver/solver.hpp"
#include "acados_solver_bombated_mpc.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"

class AcadosSolver : public SolverInterface {
public:
    AcadosSolver(const ControlParameters& params);
    ~AcadosSolver();

    void set_state(const custom_interfaces::msg::VehicleStateVector& state) override;
    void set_path(const custom_interfaces::msg::PathPointArray& path) override;
    common_lib::structures::ControlCommand solve(int* solver_status = nullptr) override; 
    
    std::vector<common_lib::structures::ControlCommand> get_full_solution() override;
    std::vector<custom_interfaces::msg::VehicleStateVector> get_full_horizon() override;
    void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) override;

private:
    void set_path_point_per_stage();
    void update_mpc_stats();
    void initialize_solver_memory();
    void print_debug_info();

    /**
     * @brief Checks if the solver output is reasonable upon solver failure, to help identify if the failure is benign (e.g. due to infeasibility) or if the solver diverged
     * 
     * @return true if the solver output is reasonable
     * @return false if the solver output is unreasonable, indicating potential divergence or numerical issues
     */
    bool sanity_check_output();

    // Acados solver components using the "bombated_mpc" prefix
    bombated_mpc_solver_capsule* capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;

    // Debug stats: Execution time of each part of the solver, for debugging and visualization purposes
    std::shared_ptr<std::vector<double>> _execution_times_;
    double average_linearization_time_ = 0.0;
    unsigned int linearization_count_ = 0;
    double average_qp_time_ = 0.0;
    unsigned int qp_count_ = 0;
    double average_regularization_time_ = 0.0;
    unsigned int regularization_count_ = 0;
    unsigned int total_sqp_iterations_ = 0;

    custom_interfaces::msg::VehicleStateVector latest_state_;
    bool has_state_ = false;
    bool has_path_ = false;
    bool is_initialized_ = false;
    std::vector<double> parameters_per_stage;

    // Debug string
    std::string stage_parameters_debug;
    std::string total_delay_debug;
};