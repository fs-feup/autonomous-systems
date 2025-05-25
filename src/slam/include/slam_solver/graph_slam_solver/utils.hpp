#pragma once

#include <gtsam/geometry/Pose2.h>

#include <Eigen/Dense>

gtsam::Pose2 eigen_to_gtsam_pose(const Eigen::Vector3d& pose);

Eigen::Vector3d gtsam_pose_to_eigen(const gtsam::Pose2& pose);