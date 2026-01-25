#pragma once

#include <map>
#include <memory>
#include <string>

#include "solver/acados.hpp"

/*
 * Map of solvers, with the key being the name of the solver and the value being a lambda
 * function that creates the solver
 */
const std::map<std::string, std::function<std::shared_ptr<SolverInterface>(const ControlParameters&)>,
               std::less<>>
    controller_map = {{"acados",
                    [](const ControlParameters& params) -> std::shared_ptr<SolverInterface> {
                      return std::make_shared<AcadosSolver>(params);
                    }}};
