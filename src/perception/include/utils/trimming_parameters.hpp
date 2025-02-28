#pragma once

struct TrimmingParameters {
  double lidar_height;        ///< LIDAR current height.
  double lidar_rotation;      ///< LIDAR rotation angle in degrees.
  double min_range;           ///< Maximum point cloud distance after trimming.
  double max_range;           ///< Minimum point cloud distance after trimming.
  double max_height;          ///< Maximum point cloud height after trimming.
  double fov_trim_angle;      ///< Maximum point cloud angle on both sides after trimming.
  double acc_max_range;       ///< (Acceleration Only) Maximum distance after trimming.
  double acc_fov_trim_angle;  ///< (Acceleration Only) Maximum angle on both sides after trimming.
  double acc_max_y;           ///< (Acceleration Only) Maximum lateral distance after trimming.
  double skid_max_range;      ///< (Skidpad Only) Maximum distance after trimming.
  double skid_min_distance_to_cone;  ///< (Skidpad Only) Min distance to a cone for it to be seen.

  TrimmingParameters() = default;
};
