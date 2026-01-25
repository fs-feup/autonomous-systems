#pragma once

/**
 * @struct TrimmingParameters
 * @brief Structure to hold parameters for trimming point cloud data
 */
struct TrimmingParameters {
  double lidar_height;                 ///< LIDAR current height.
  double lidar_horizontal_resolution;  ///< LIDAR horizontal resolution.
  double lidar_vertical_resolution;    ///< LIDAR vertical resolution.
  bool apply_rotation;                 ///< Whether to apply rotation to the point cloud.
  double rotation;                     ///< Rotation angle to be applied to the point cloud.
  bool apply_fov_trimming;             ///< Whether to apply field of view trimming.
  double fov;                          ///< Field of view.
  bool is_raining;                     ///< Whether it is raining.
  double max_height;                   ///< Maximum point cloud height after trimming.
  double min_range;                    ///< Maximum point cloud distance after trimming.
  double max_range;                    ///< Minimum point cloud distance after trimming.
  double acc_max_range;                ///< (Acceleration Only) Maximum distance after trimming.
  double acc_max_y;       ///< (Acceleration Only) Maximum lateral distance after trimming.
  double skid_max_range;  ///< (Skidpad Only) Maximum distance after trimming.

  TrimmingParameters() = default;
};
