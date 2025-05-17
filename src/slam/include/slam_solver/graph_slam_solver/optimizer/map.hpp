#pragma once

#include <map>
#include <memory>
#include <string>

#include "slam_solver/graph_slam_solver/optimizer/base_optimizer.hpp"
#include "slam_solver/graph_slam_solver/optimizer/isam2_optimizer.hpp"
#include "slam_solver/graph_slam_solver/optimizer/normal_levenberg_optimizer.hpp"
#include "slam_solver/graph_slam_solver/optimizer/sliding_window_levenberg_optimizer.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string, std::function<std::shared_ptr<BaseOptimizer>(const SLAMParameters&)>,
               std::less<>>
    graph_slam_optimizer_constructors_map = {
        {"normal_levenberg",
         [](const SLAMParameters& params) -> std::shared_ptr<BaseOptimizer> {
           return std::make_shared<NormalLevenbergOptimizer>(params);
         }},
        {"sliding_window_levenberg",
         [](const SLAMParameters& params) -> std::shared_ptr<BaseOptimizer> {
           return std::make_shared<SlidingWindowLevenbergOptimizer>(params);
         }},
        {"isam2",
         [](const SLAMParameters& params) -> std::shared_ptr<BaseOptimizer> {
           return std::make_shared<ISAM2Optimizer>(params);
         }},
};
