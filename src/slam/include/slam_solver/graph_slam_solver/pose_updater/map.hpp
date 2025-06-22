#pragma once

#include <map>
#include <memory>
#include <string>

#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/linear_fusion_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/odometry_based_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/velocities_based_pose_updater.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string, std::function<std::shared_ptr<PoseUpdater>()>, std::less<>>
    graph_slam_optimizer_constructors_map = {
        {"velocities_based",
         []() -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<VelocitiesBasedPoseUpdater>();
         }},
        {"linear_fusion",
         []() -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<LinearFusionPoseUpdater>();
         }},

        {"odometry_based",
         []() -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<OdometryBasedPoseUpdater>();
         }},
};
