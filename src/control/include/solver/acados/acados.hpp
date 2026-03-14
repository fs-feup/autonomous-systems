#pragma once

#include "solver/solver.hpp"
#include "c_generated_code/acados_solver_mpc.h"
#include "std_msgs/msg/float64_multi_array.hpp"

class AcadosSolver : public SolverInterface {
public:
    AcadosSolver(const ControlParameters& params);
    ~AcadosSolver();

    void set_state(const std::vector<double>& x0) override;
    void set_path(const std::vector<double>& x_path) override;
    common_lib::structures::ControlCommand solve() override; 
    
    std::vector<common_lib::structures::ControlCommand> get_full_solution() override;
    std::vector<custom_interfaces::msg::VehicleStateVector> get_full_horizon() override;
    void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) override;

private:
    void set_path_point_per_stage();
    void update_execution_times();

    // Acados solver components using the "mpc" prefix
    mpc_solver_capsule* capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;

    // Execution time of each part of the solver, for debugging and visualization purposes
    std::shared_ptr<std::vector<double>> _execution_times_;
    double average_linearization_time_ = 0.0;
    unsigned int linearization_count_ = 0;
    double average_qp_time_ = 0.0;
    unsigned int qp_count_ = 0;
    double average_regularization_time_ = 0.0;
    unsigned int regularization_count_ = 0;

    std::vector<double> last_state_;
    std::vector<double> last_path_;
    bool has_state_ = false;
    bool has_path_ = false;
};