#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "utils/grid_geometry.hpp"

class GroundGrid {
public:
  GroundGrid() = default;

  GroundGrid(const double range, const double angle, const double radius,
             const double start_augmentation, const double radius_augmentation, const double fov);

  float get_ground_height(const float x, const float y) const;
  void set_ground_height(const float x, const float y, const float height);
  void reset_grid();

private:
  double range_;
  GridGeometry grid_geometry_;
  std::vector<std::vector<float>> grid_;
};
