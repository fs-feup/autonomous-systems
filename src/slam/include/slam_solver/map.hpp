#pragma once

#include <map>
#include <memory>
#include <string>

#include "slam_solver/ekf_slam_solver.hpp"
#include "slam_solver/graph_slam_solver.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string,
               std::function<std::shared_ptr<SLAMSolver>(
                   const SLAMParameters&, std::shared_ptr<DataAssociationModel>,
                   std::shared_ptr<V2PMotionModel>, std::shared_ptr<LandmarkFilter>,
                   std::shared_ptr<std::vector<double>>, std::weak_ptr<rclcpp::Node>)>,
               std::less<>>
    slam_solver_constructors_map = {
        {"graph_slam",
         [](const SLAMParameters& params, std::shared_ptr<DataAssociationModel> data_association,
            std::shared_ptr<V2PMotionModel> motion_model,
            std::shared_ptr<LandmarkFilter> landmark_filter,
            std::shared_ptr<std::vector<double>> execution_times,
            std::weak_ptr<rclcpp::Node> node) -> std::shared_ptr<SLAMSolver> {
           return std::make_shared<GraphSLAMSolver>(params, data_association, motion_model,
                                                    landmark_filter, execution_times, node);
         }},
        {"ekf_slam",
         [](const SLAMParameters& params, std::shared_ptr<DataAssociationModel> data_association,
            std::shared_ptr<V2PMotionModel> motion_model,
            std::shared_ptr<LandmarkFilter> landmark_filter,
            std::shared_ptr<std::vector<double>> execution_times,
            std::weak_ptr<rclcpp::Node> node) -> std::shared_ptr<SLAMSolver> {
           return std::make_shared<EKFSLAMSolver>(params, data_association, motion_model,
                                                  landmark_filter, execution_times, node);
         }},
};
