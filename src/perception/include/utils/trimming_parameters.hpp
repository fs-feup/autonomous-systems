#pragma once

struct TrimmingParameters {
  double lidar_height;          ///< LIDAR current height.
  double min_range;             ///< Maximum point cloud distance after trimming.
  double max_range;             ///< Minimum point cloud distance after trimming.
  double max_height;            ///< Maximum point cloud height after trimming.
  double fov_trim_angle;        ///< Maximum point cloud angle on both sides after trimming.
  double acc_max_y;             ///< (Acceleration Only) Maximum lateral distance after trimming.
  double min_distance_to_cone;  ///< (Skidpad Only) Minimum distance to a cone for it to be seen.

  TrimmingParameters() = default;
  void set_acceleration();
  void set_skidpad();
};
