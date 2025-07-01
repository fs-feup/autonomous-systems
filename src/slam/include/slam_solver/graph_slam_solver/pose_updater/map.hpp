#pragma once

#include <map>
#include <memory>
#include <string>

#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/double_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/odometry_based_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/velocities_based_pose_updater.hpp"

/*
 * Map of slam_solvers, with the key being the type of the slam_solver and the value being a lambda
 * function that returns a shared pointer to the corresponding slam_solver
 */
const std::map<std::string, std::function<std::shared_ptr<PoseUpdater>(const SLAMParameters&)>,
               std::less<>>
    pose_updater_constructors_map = {
        {"velocities_based",
         [](const SLAMParameters& params) -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<VelocitiesBasedPoseUpdater>(params);
         }},
        {"double pose_updater",
         [](const SLAMParameters& params) -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<DoublePoseUpdater>(params);
         }},
        {"odometry_based",
         [](const SLAMParameters& params) -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<OdometryBasedPoseUpdater>(params);
         }},
};
