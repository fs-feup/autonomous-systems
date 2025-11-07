#pragma once

#include <cmath>

struct GridGeometry {
  double angle;                // Angular resolution (°)
  double radius;               // Base radial bin size (m)
  double start_augmentation;   // Distance at which augmentation starts
  double radius_augmentation;  // Bin size increase per bin (m)
  double fov;                  // Field of view (°)

  GridGeometry(double angle_, double radius_, double start_aug_, double radius_aug_, double fov_);

  // Compute slice index for a given (x, y)
  int get_slice_index(double x, double y) const;

  // Compute bin index for a given (x, y)
  int get_bin_index(double x, double y) const;

  int get_num_slices() const;

  int get_num_bins(double range) const;

  int get_num_constant_bins() const;
};