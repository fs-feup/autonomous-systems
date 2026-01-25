#pragma once

#include "solver/solver.hpp"
#include "c_generated_code/acados_solver_mpc.h"

class AcadosSolver : public SolverInterface {
public:
    AcadosSolver(const ControlParameters& params);
    ~AcadosSolver();

    void set_state(const std::vector<double>& x0) override;
    void set_path(const std::vector<double>& x_path) override;
    common_lib::structures::ControlCommand solve() override; 
    
    std::vector<common_lib::structures::ControlCommand> get_full_solution() override;
    std::vector<std::pair<common_lib::structures::Pose, common_lib::structures::VehicleState>> get_full_horizon() override;

private:
    // Acados solver components using the "mpc" prefix
    mpc_solver_capsule* capsule_;
    ocp_nlp_config* nlp_config_;
    ocp_nlp_dims* nlp_dims_;
    ocp_nlp_in* nlp_in_;
    ocp_nlp_out* nlp_out_;
};