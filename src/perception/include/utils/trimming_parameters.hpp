#pragma once

#include <utils/split_parameters.hpp>

#include "common_lib/competition_logic/mission_logic.hpp"

struct TrimmingParameters {
  common_lib::competition_logic::Mission current_mission_type;

  double lidar_height;    ///< LIDAR current height.
  double lidar_rotation;  ///< LIDAR rotation angle in degrees.
  double lidar_pitch;     ///< LIDAR pitch angle in degrees.
  double max_height;      ///< Maximum point cloud height after trimming.
  double min_range;       ///< Maximum point cloud distance after trimming.

  double max_range;              ///< Minimum point cloud distance after trimming.
  double fov_trim_angle;         ///< Maximum point cloud angle on both sides after trimming.
  SplitParameters split_params;  ///< Split parameters for GridRANSAC.

  double acc_max_range;       ///< (Acceleration Only) Maximum distance after trimming.
  double acc_fov_trim_angle;  ///< (Acceleration Only) Maximum angle on both sides after trimming.
  double acc_max_y;           ///< (Acceleration Only) Maximum lateral distance after trimming.
  SplitParameters acc_split_params;  ///< (Acceleration Only) Split parameters for GridRANSAC.

  double skid_max_range;       ///< (Skidpad Only) Maximum distance after trimming.
  double skid_fov_trim_angle;  ///< (Skidpad Only) Maximum angle on both sides after trimming.
  SplitParameters skid_split_params;  ///< (Skidpad Only) Split parameters for GridRANSAC.

  TrimmingParameters() = default;
};
