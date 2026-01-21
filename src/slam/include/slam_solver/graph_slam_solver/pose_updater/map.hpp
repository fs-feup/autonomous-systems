#pragma once

#include <map>
#include <memory>
#include <string>

#include "slam_solver/graph_slam_solver/pose_updater/base_pose_updater.hpp"
#include "slam_solver/graph_slam_solver/pose_updater/difference_based_ready_pose_updater.hpp"

/*
 * Map of pose updaters, with the key being the type of the pose updater and the value being a
 * lambda function that returns a shared pointer to the corresponding pose updater
 */
const std::map<std::string, std::function<std::shared_ptr<PoseUpdater>(const SLAMParameters&)>,
               std::less<>>
    pose_updater_constructors_map = {
        {"difference_based_ready",
         [](const SLAMParameters& params) -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<DifferenceBasedReadyPoseUpdater>(params);
         }},
        {"base_pose_updater",
         [](const SLAMParameters& params) -> std::shared_ptr<PoseUpdater> {
           return std::make_shared<PoseUpdater>(params);
         }},
};
