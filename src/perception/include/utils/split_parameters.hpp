#pragma once

struct SplitParameters {
  double fov_angle = 0;          ///< field of view angle of the pcl after trimming.
  int n_angular_grids = 0;       ///< Number of angular grids in the algorithm.
  double radius_resolution = 0;  ///< Resolution of the radius for grid splitting.
  double angle_resolution = 0;   ///< Resolution of the angle for slice splitting in Himmelsbach.
  double lidar_horizontal_resolution = 0;  ///< Horizontal angular resolution of the LiDAR sensor.
  double max_range = 0;          ///< Maximum range of the pcl after trimming.

  SplitParameters() = default;
};
