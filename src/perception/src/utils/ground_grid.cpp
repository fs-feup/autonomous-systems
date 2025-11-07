#include "utils/ground_grid.hpp"

GroundGrid::GroundGrid(const double range, const double angle, const double radius,
                       const double start_augmentation, const double radius_augmentation,
                       const double fov)
    : range_(range), grid_geometry_(angle, radius, start_augmentation, radius_augmentation, fov) {
  int num_slices_ = grid_geometry_.get_num_slices();
  int num_bins_ = grid_geometry_.get_num_bins(range_);
  grid_.resize(num_slices_, std::vector<float>(num_bins_, std::numeric_limits<float>::quiet_NaN()));
}

float GroundGrid::get_ground_height(const float x, const float y) const {
  int slice = grid_geometry_.get_slice_index(x, y);
  int bin_idx = grid_geometry_.get_bin_index(x, y);
  int num_slices_ = grid_geometry_.get_num_slices();
  int num_bins_ = grid_geometry_.get_num_bins(range_);

  if (slice < 0 || slice >= num_slices_ || bin_idx < 0 || bin_idx >= num_bins_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  return grid_[slice][bin_idx];
}

void GroundGrid::set_ground_height(const float x, const float y, const float height) {
  int slice = grid_geometry_.get_slice_index(x, y);
  int bin_idx = grid_geometry_.get_bin_index(x, y);
  int num_slices_ = grid_geometry_.get_num_slices();
  int num_bins_ = grid_geometry_.get_num_bins(range_);

  if (slice < 0 || slice >= num_slices_ || bin_idx < 0 || bin_idx >= num_bins_) {
    return;  // out of bounds, ignore
  }

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
