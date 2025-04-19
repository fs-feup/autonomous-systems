#pragma once

struct EvaluatorResults {
  double height_in_ratio_large;
  double height_out_ratio_large;
  double height_out_ratio_small;
  double height_in_ratio_small;
  bool height_large;

  double cylinder_out_distance_xy_large;
  double cylinder_out_distance_z_large;
  double cylinder_out_distance_xy_small;
  double cylinder_out_distance_z_small;
  double cylinder_n_large_points;
  double cylinder_n_out_points;

  double deviation_xoy;
  double deviation_z;

  double displacement_x;
  double displacement_y;
  double displacement_z;

  double n_points;
};
