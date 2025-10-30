#pragma once
struct SplitParameters {
  double lidar_height;                 ///< Height of the LIDAR from the ground in meters.
  double lidar_horizontal_resolution;  ///< LIDAR horizontal angular resolution in degrees.
  double angle_resolution;             ///< Desired angular resolution for splitting in degrees.
  double fov;                          ///< Field of view of the LIDAR in degrees.

  SplitParameters() = default;
};
