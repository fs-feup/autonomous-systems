#include "solver/acados/acados.hpp"

#include <cmath>
#include <limits>

namespace {

constexpr int kPathSize = 50;
constexpr int kPathPointSize = 4;
constexpr double kWeightEps = 0.5;

struct WeightDiagnostics {
    bool valid = false;
    double min_dist_sq = std::numeric_limits<double>::quiet_NaN();
    double raw_sum = std::numeric_limits<double>::quiet_NaN();
    double shifted_sum = std::numeric_limits<double>::quiet_NaN();
    int finite_points = 0;
};

WeightDiagnostics compute_weight_diagnostics(const std::vector<double>& state, const std::vector<double>& path) {
    WeightDiagnostics diagnostics;
    if (state.size() < 2 || path.size() != static_cast<size_t>(kPathSize * kPathPointSize)) {
        return diagnostics;
    }

    const double car_x = state[0];
    const double car_y = state[1];

    std::vector<double> dist_sq(kPathSize, 0.0);
    diagnostics.min_dist_sq = std::numeric_limits<double>::infinity();

    for (int i = 0; i < kPathSize; ++i) {
        const double px = path[i * kPathPointSize];
        const double py = path[i * kPathPointSize + 1];
        if (!std::isfinite(px) || !std::isfinite(py)) {
            continue;
        }

        const double dx = px - car_x;
        const double dy = py - car_y;
        const double d2 = dx * dx + dy * dy;
        dist_sq[i] = d2;
        diagnostics.min_dist_sq = std::min(diagnostics.min_dist_sq, d2);
        diagnostics.finite_points++;
    }

    if (diagnostics.finite_points == 0 || !std::isfinite(diagnostics.min_dist_sq)) {
        return diagnostics;
    }

    diagnostics.raw_sum = 0.0;
    diagnostics.shifted_sum = 0.0;
    for (int i = 0; i < kPathSize; ++i) {
        const double px = path[i * kPathPointSize];
        const double py = path[i * kPathPointSize + 1];
        if (!std::isfinite(px) || !std::isfinite(py)) {
            continue;
        }

        const double d2 = dist_sq[i];
        diagnostics.raw_sum += std::exp(-d2 / kWeightEps);
        diagnostics.shifted_sum += std::exp(-(d2 - diagnostics.min_dist_sq) / kWeightEps);
    }

    diagnostics.valid = true;
    return diagnostics;
}

}  // namespace

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

    last_state_ = x0;
    has_state_ = true;

    // Set the initial state constraint (lbx and ubx) at stage 0
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", (void*)x0.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", (void*)x0.data());
}

void AcadosSolver::set_path(const std::vector<double>& x_path) {
    if (x_path.size() != static_cast<size_t>(kPathSize * kPathPointSize)) {
        RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "Path size mismatch: expected %d, got %zu", kPathSize * kPathPointSize, x_path.size());
    }

    last_path_ = x_path;
    has_path_ = true;

    mpc_acados_set_p_global_and_precompute_dependencies(capsule_, (double*)x_path.data(), x_path.size());
}

common_lib::structures::ControlCommand AcadosSolver::solve() {
    common_lib::structures::ControlCommand command;

    int status = mpc_acados_solve(capsule_);

    if (status != ACADOS_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "MPC Solve Failed! Status: %d", status);

        if (has_state_ && has_path_) {
            const WeightDiagnostics diagnostics = compute_weight_diagnostics(last_state_, last_path_);
            if (diagnostics.valid) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("AcadosSolver"),
                    "external cost weight diagnostics: min_dist_sq=%.6f raw_sum_W=%.6e shifted_sum=%.6e finite_points=%d",
                    diagnostics.min_dist_sq,
                    diagnostics.raw_sum,
                    diagnostics.shifted_sum,
                    diagnostics.finite_points);

                if (diagnostics.raw_sum == 0.0) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("AcadosSolver"),
                        "raw sum(weights) is exactly zero -> likely underflow in exp(-distance/eps) causing NaN in external cost.");
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Could not compute weight diagnostics (invalid state/path cache).");
            }
        } else {
            RCLCPP_ERROR(
                rclcpp::get_logger("AcadosSolver"),
                "Weight diagnostics unavailable: has_state=%d has_path=%d",
                has_state_,
                has_path_);
        }

        ocp_nlp_solver* nlp_solver = mpc_acados_get_nlp_solver(capsule_);
        int sqp_iter = -1;
        int qp_iter = -1;
        int qp_status = -1;
        double time_tot = std::numeric_limits<double>::quiet_NaN();
        double kkt_norm_inf = std::numeric_limits<double>::quiet_NaN();

        ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
        ocp_nlp_get(nlp_solver, "qp_iter", &qp_iter);
        ocp_nlp_get(nlp_solver, "qp_status", &qp_status);
        ocp_nlp_get(nlp_solver, "time_tot", &time_tot);
        ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);

        RCLCPP_ERROR(
            rclcpp::get_logger("AcadosSolver"),
            "acados diagnostics: sqp_iter=%d qp_iter=%d qp_status=%d time_tot=%.6e kkt_inf=%.6e",
            sqp_iter,
            qp_iter,
            qp_status,
            time_tot,
            kkt_norm_inf);

        mpc_acados_print_stats(capsule_);
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

