#pragma once

#include <map>
#include <memory>
#include <string>

#include "solver/bombated_mpc_acados/bombated_mpc_acados.hpp"
#include "solver/mpczinho_acados/mpczinho_acados.hpp"
#include "solver/supermpc_acados/supermpc_acados.hpp"

/*
 * Map of solvers, with the key being the name of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<SolverInterface>(const ControlParameters&)>,
               std::less<>>
    solver_map = {{"bombated_mpc_acados",
                    [](const ControlParameters& params) -> std::shared_ptr<SolverInterface> {
                      return std::make_shared<AcadosSolver>(params);
                    }},
                  {"mpczinho_acados",
                   [](const ControlParameters& params) -> std::shared_ptr<SolverInterface> {
                     return std::make_shared<MPCzinhoAcadosSolver>(params);
                   }},
                  {"supermpc_acados",
                   [](const ControlParameters& params) -> std::shared_ptr<SolverInterface> {
                     return std::make_shared<SuperMpcAcadosSolver>(params);
                   }}
                  };
