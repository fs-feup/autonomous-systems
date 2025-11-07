#include "utils/ground_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

GroundGrid::GroundGrid(const double range, const double angle, const double radius,
                       const double start_augmentation, const double radius_augmentation,
                       const double fov)
    : range_(range),
      angle_(angle),
      radius_(radius),
      start_augmentation_(start_augmentation),
      radius_augmentation_(radius_augmentation),
      fov_(fov) {
  num_slices_ = static_cast<int>(std::ceil(fov / angle_));
  num_bins_ = static_cast<int>(std::ceil(range_ / radius_));
  base_bins_ = static_cast<int>(std::floor(start_augmentation_ / radius_));
  const_end_ = base_bins_ * radius_;
  grid_.resize(num_slices_, std::vector<float>(num_bins_, std::numeric_limits<float>::quiet_NaN()));
}

float GroundGrid::get_ground_height(const float x, const float y) const {
  // --- Convert to slice index ---
  int slice = static_cast<int>(std::floor((std::atan2(y, x) * 180.0 / M_PI + (fov_)) / angle_));

  // --- Compute distance from origin ---
  double distance = std::sqrt(x * x + y * y);
  int bin_idx = 0;

  if (distance < start_augmentation_ || radius_augmentation_ == 0.0) {
    // Constant-size bins region
    bin_idx = static_cast<int>(std::floor(distance / radius_));
  } else {
    // --- Augmented bins region (closed-form) ---
    const double d = distance - start_augmentation_;
    const double a = radius_augmentation_ / 2.0;
    const double b = radius_ + radius_augmentation_ / 2.0;
    const double disc = b * b + 4.0 * a * d;

    // Positive root of quadratic
    const double n_pos = (-b + std::sqrt(disc)) / (2.0 * a);

    // Convert to integer bin number (smallest integer n with S_n > d)
    const int n = static_cast<int>(std::floor(n_pos)) + 1;

    bin_idx = base_bins_ + (n - 1);
  }

  // --- Sanity check for bounds ---
  if (slice < 0 || slice >= num_slices_ || bin_idx < 0 || bin_idx >= num_bins_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  // --- Return stored ground height ---
  return grid_[slice][bin_idx];
}

void GroundGrid::set_ground_height(const float x, const float y, const float height) {
  // --- Compute slice index (angular division) ---
  int slice = static_cast<int>(std::floor((std::atan2(y, x) * 180.0 / M_PI + (fov_)) / angle_));

  // --- Compute radial distance ---
  double distance = std::sqrt(x * x + y * y);
  int bin_idx = 0;

  // --- Constant bin region ---
  if (distance < start_augmentation_ || radius_augmentation_ == 0.0) {
    bin_idx = static_cast<int>(std::floor(distance / radius_));
  } else {
    // --- Augmented bin region (closed form) ---
    const double d = distance - start_augmentation_;
    const double a = radius_augmentation_ / 2.0;
    const double b = radius_ + radius_augmentation_ / 2.0;
    const double disc = b * b + 4.0 * a * d;
    const double n_pos = (-b + std::sqrt(disc)) / (2.0 * a);
    const int n = static_cast<int>(std::floor(n_pos)) + 1;
    bin_idx = base_bins_ + (n - 1);
  }

  // --- Bounds check ---
  if (slice < 0 || slice >= num_slices_ || bin_idx < 0 || bin_idx >= num_bins_) {
    return;  // out of bounds, ignore
  }

  // --- Update grid value ---
  if (std::isnan(grid_[slice][bin_idx])) {
    grid_[slice][bin_idx] = height;
  } else {
    // Update only if new height is lower
    grid_[slice][bin_idx] = std::min(grid_[slice][bin_idx], height);
  }
}

void GroundGrid::reset_grid() {
  for (auto& row : grid_) {
    std::fill(row.begin(), row.end(), std::numeric_limits<float>::quiet_NaN());
  }
}
