#pragma once

#include <cone_validator/cone_validator.hpp>

// struct EvaluatorParameters {
//   std::shared_ptr<ConeValidator> npoints_validator;
//   std::shared_ptr<ConeValidator> height_validator;
//   std::shared_ptr<ConeValidator> cylinder_validator;
//   std::shared_ptr<ConeValidator> deviation_validator;
//   std::shared_ptr<ConeValidator> displacement_validator;
//   std::shared_ptr<ConeValidator> zscore_validator;
//
//   // Height weights
//   double height_out_weight = 0.0;
//   double height_in_weight = 0.0;
//
//   // Cylinder weights
//   double cylinder_radius_weight = 0.0;
//   double cylinder_height_weight = 0.0;
//   double cylinder_npoints_weight = 0.0;
//   double npoints_weight = 0.0;
//
//   // Displacement weights
//   double displacement_x_weight = 0.0;
//   double displacement_y_weight = 0.0;
//   double displacement_z_weight = 0.0;
//
//   // Deviation weights
//   double deviation_xoy_weight = 0.0;
//   double deviation_z_weight = 0.0;
//
//   double min_confidence = 1.0;
//
//   EvaluatorParameters() = default;
//   double getSum() const;
//   void normalize_weights();
// };

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
  double lidar_height;
  double lidar_vertical_resolution;
  double lidar_horizontal_resolution;
  double visibility_factor;
  double expected_points_threshold;
};
