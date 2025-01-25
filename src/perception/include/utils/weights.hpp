#pragma once

struct Weights {
  // Height weights
  double height_out = 0.0;
  double height_in = 0.0;

  // Cylinder weights
  double cylinder_radius = 0.0;
  double cylinder_height = 0.0;
  double cylinder_npoints = 0.0;
  double npoints = 0.0;

  // Displacement weights
  double displacement_x = 0.0;
  double displacement_y = 0.0;
  double displacement_z = 0.0;

  // Deviation weights
  double deviation_xoy = 0.0;
  double deviation_z = 0.0;

  Weights() = default;
  double getSum() const;
  void normalize();
};
