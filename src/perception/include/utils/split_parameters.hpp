#pragma once
struct SplitParameters {
  double max_distance = 0;       ///< Maximum distance of the pcl after trimming.
  double fov_angle = 0;          ///< field of view angle of the pcl after trimming.
  int n_angular_grids = 0;       ///< Number of angular grids in the algorithm.
  double radius_resolution = 0;  ///< Resolution of the radius for grid splitting.

  SplitParameters() = default;
};
