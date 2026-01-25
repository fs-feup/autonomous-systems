#include "solver/acados/acados.hpp"

AcadosSolver::AcadosSolver(const ControlParameters& params) : SolverInterface(params) {
    // 1. Create the capsule
    capsule_ = mpc_acados_create_capsule();
    
    // 2. Allocate solver memory
    int status = mpc_acados_create(capsule_);
    if (status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Failed to create Acados solver 'mpc', status: %d", status);
    }

    // 3. Cache internal pointers
    nlp_config_ = mpc_acados_get_nlp_config(capsule_);
    nlp_dims_ = mpc_acados_get_nlp_dims(capsule_);
    nlp_in_ = mpc_acados_get_nlp_in(capsule_);
    nlp_out_ = mpc_acados_get_nlp_out(capsule_);
}

AcadosSolver::~AcadosSolver() {
    mpc_acados_free(capsule_);
    mpc_acados_free_capsule(capsule_);
}

void AcadosSolver::set_state(const std::vector<double>& x0) {
    if (x0.size() != 13) {
        RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "State size mismatch: expected 13, got %zu", x0.size());
        return;
    }

    // Set the initial state constraint (lbx and ubx) at stage 0
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", (void*)x0.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", (void*)x0.data());
}

void AcadosSolver::set_path(const std::vector<double>& x_path) {
    mpc_acados_set_p_global_and_precompute_dependencies(capsule_, (double*)x_path.data(), x_path.size());
}

common_lib::structures::ControlCommand AcadosSolver::solve() {
    common_lib::structures::ControlCommand command;

    int status = mpc_acados_solve(capsule_);

    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "MPC Solve Failed! Status: %d", status);
        return command; 
    }

    // Extracting 3 controls
    double solved_controls[3]; 
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", (void*)solved_controls);

    // Map the controls to our command
    command.throttle_rl= solved_controls[0];
    command.throttle_rr= solved_controls[1];
    command.steering_angle = solved_controls[2];
    
    return command;
}



std::vector<common_lib::structures::ControlCommand> AcadosSolver::get_full_solution() {
    int N = nlp_dims_->N;
    std::vector<common_lib::structures::ControlCommand> full_u;
    
    for (int i = 0; i < N; ++i) {
        double solved_controls[3]; 
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", (void*)solved_controls);
        
        common_lib::structures::ControlCommand cmd;
        cmd.throttle_rl= solved_controls[0]; 
        cmd.throttle_rr= solved_controls[1];
        cmd.steering_angle = solved_controls[2];
        full_u.push_back(cmd);
    }
    return full_u;
}

std::vector<std::pair<common_lib::structures::Pose, common_lib::structures::VehicleState>> AcadosSolver::get_full_horizon() {
    // Similar to get_full_solution, but extracting "x" for stages 0 to N
    return {}; 
}

