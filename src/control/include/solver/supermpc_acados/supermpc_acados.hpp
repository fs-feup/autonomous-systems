#pragma once

#include "solver/solver.hpp"
#include "acados_solver_supermpc.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "custom_interfaces/msg/vehicle_state_vector.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "common_lib/communication/marker.hpp"

class SuperMpcAcadosSolver : public SolverInterface {
public:
    SuperMpcAcadosSolver(const ControlParameters& params);
    ~SuperMpcAcadosSolver();

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
    void apply_cost_weights();
    void reset_solver();
    void apply_envelope();
    void publish_interpolated_path(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map);
    void publish_received_state(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map);

    /**
     * @brief Checks if the solver output is reasonable upon solver failure, to help identify if the failure is benign (e.g. due to infeasibility) or if the solver diverged
     * 
     * @return true if the solver output is reasonable
     * @return false if the solver output is unreasonable, indicating potential divergence or numerical issues
     */
    bool sanity_check_output();

    // Acados solver components using the "supermpc" prefix
    supermpc_solver_capsule* capsule_;
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

    // The throttle/steering commands are integrated states of the model, and the
    // plant never reports them back, so the solver carries them between cycles.
    double last_throttle_command_ = 0.0;
    double last_steering_command_ = 0.0;
    // Crude observer for the inverter's transport delay, advanced once per
    // control period (25 ms) towards the command with the model's 0.2 s lag.
    static constexpr double kInverterBlend = 0.025 / 0.2;
    double applied_throttle_estimate_ = 0.0;
    int consecutive_failures_ = 0;
    rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

    // Debug string
    std::string stage_parameters_debug;
    std::string total_delay_debug;
};