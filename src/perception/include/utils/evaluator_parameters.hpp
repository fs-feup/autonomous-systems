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

  // Number of points parameters
  double n_points_intial_max;
  double n_points_intial_min;
  double n_points_max_distance_reduction;
  double n_points_min_distance_reduction;
};
