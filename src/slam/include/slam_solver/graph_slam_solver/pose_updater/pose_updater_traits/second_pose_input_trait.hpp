#pragma once

#include <Eigen/Dense>

class SecondPoseInputTrait {
  int _last_pose_number_ = 0;

public:
  virtual Eigen::Vector3d get_second_accumulated_pose_difference() const = 0;

  virtual bool second_pose_difference_ready() const = 0;
};