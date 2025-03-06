#pragma once

#include <utils/pcl_split_parameters.hpp>

struct TrimmingParameters {
  double lidar_height;    ///< LIDAR current height.
  double lidar_rotation;  ///< LIDAR rotation angle in degrees.
  double lidar_pitch;     ///< LIDAR pitch angle in degrees.
  double max_height;      ///< Maximum point cloud height after trimming.
  double min_range;       ///< Maximum point cloud distance after trimming.

  double max_range;                 ///< Minimum point cloud distance after trimming.
  double fov_trim_angle;            ///< Maximum point cloud angle on both sides after trimming.
  PclSplitParameters split_params;  ///< Split parameters for GridRANSAC.

  double acc_max_range;       ///< (Acceleration Only) Maximum distance after trimming.
  double acc_fov_trim_angle;  ///< (Acceleration Only) Maximum angle on both sides after trimming.
  double acc_max_y;           ///< (Acceleration Only) Maximum lateral distance after trimming.
  PclSplitParameters acc_split_params;  ///< (Acceleration Only) Split parameters for GridRANSAC.

  double skid_max_range;       ///< (Skidpad Only) Maximum distance after trimming.
  double skid_fov_trim_angle;  ///< (Skidpad Only) Maximum angle on both sides after trimming.
  PclSplitParameters skid_split_params;  ///< (Skidpad Only) Split parameters for GridRANSAC.

  TrimmingParameters() = default;
};
