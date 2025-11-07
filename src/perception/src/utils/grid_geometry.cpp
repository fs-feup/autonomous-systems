#include "utils/grid_geometry.hpp"

GridGeometry::GridGeometry(double angle_, double radius_, double start_aug_, double radius_aug_,
                           double fov_)
    : angle(angle_),
      radius(radius_),
      start_augmentation(start_aug_),
      radius_augmentation(radius_aug_),
      fov(fov_) {}

int GridGeometry::get_slice_index(double x, double y) const {
  // Bags rotated by drivers, what shoud be used with current lidar
  double angle_deg = std::atan2(y, x) * 180.0 / M_PI;  // [-90, 90]

  // Bags not rotated by drivers
  // double angle_deg = std::atan2(x, -y) * 180.0 / M_PI;  // [-90, 90]
  angle_deg += fov / 2;  // Convert to [0, fov]

  if (angle_deg < 0.0 || angle_deg >= fov) {
    return -1;  // Out of FOV
  }

  return static_cast<int>(std::floor(angle_deg / angle));
}

int GridGeometry::get_bin_index(double x, double y) const {
  double distance = std::sqrt(x * x + y * y);
  int bin_idx = 0;

  if (distance < start_augmentation || radius_augmentation == 0.0) {
    bin_idx = static_cast<int>(std::floor(distance / radius));
  } else {
    const double d = distance - start_augmentation;
    const double a = radius_augmentation / 2.0;
    const double b = radius + radius_augmentation / 2.0;
    const double disc = b * b + 4.0 * a * d;
    const double n_pos = (-b + std::sqrt(disc)) / (2.0 * a);
    const int n = static_cast<int>(std::floor(n_pos)) + 1;
    bin_idx = static_cast<int>(std::floor(start_augmentation / radius)) + (n - 1);
  }

  return std::max(bin_idx, 0);
}

int GridGeometry::get_num_slices() const {
  return std::max(1, static_cast<int>(std::ceil(fov / angle)));
}

int GridGeometry::get_num_bins(double range) const {
  int num_bins = 0;
  if (radius_augmentation == 0.0 || range <= start_augmentation) {
    num_bins = static_cast<int>(std::ceil(range / radius));
  } else {
    int base_bins = static_cast<int>(std::floor(start_augmentation / radius));
    num_bins = base_bins;

    double dist_remaining = range - start_augmentation;
    double current_bin_size = radius;

    while (dist_remaining > 0) {
      dist_remaining -= current_bin_size;
      if (dist_remaining >= 0) {
        num_bins++;
      }
      current_bin_size += radius_augmentation;
    }
  }
  return num_bins;
}

int GridGeometry::get_num_constant_bins() const {
  return static_cast<int>(std::floor(start_augmentation / radius));
}