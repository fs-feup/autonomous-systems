struct PclSplitParameters {
  double fov_angle;          ///< field of view angle of the pcl after trimming.
  double n_angular_grids;    ///< Number of angular grids in the algorithm.
  double radius_resolution;  ///< Resolution of the radius for grid splitting.

  PclSplitParameters() = default;
};
