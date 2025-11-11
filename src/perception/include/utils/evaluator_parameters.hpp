#pragma once

/**
 * @struct EvaluatorParameters
 *
 * @brief Structure to hold parameters for evaluating clusters as cones
 */
struct EvaluatorParameters {
  // Cone size parameters
  double small_cone_width;
  double large_cone_width;
  double small_cone_height;
  double large_cone_height;

  // Cylinder fit parameters
  double n_out_points_ratio;

  // Ground proximity parameters
  double max_distance_from_ground_min;
  double max_distance_from_ground_max;

  // Number of points parameters, not used currently
  double lidar_height;
  double lidar_vertical_resolution;
  double lidar_horizontal_resolution;
  double visibility_factor;
  double expected_points_threshold;
};
