#pragma once

struct TrimmingParameters {
  double lidar_height;    ///< LIDAR current height.
  double max_height;      ///< Maximum point cloud height after trimming.
  double min_range;       ///< Maximum point cloud distance after trimming.
  double max_range;       ///< Minimum point cloud distance after trimming.
  double acc_max_range;   ///< (Acceleration Only) Maximum distance after trimming.
  double acc_max_y;       ///< (Acceleration Only) Maximum lateral distance after trimming.
  double skid_max_range;  ///< (Skidpad Only) Maximum distance after trimming.

  TrimmingParameters() = default;
};
