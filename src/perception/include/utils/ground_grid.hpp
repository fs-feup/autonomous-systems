#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "utils/grid_geometry.hpp"

/**
 * @class GroundGrid
 *
 * @brief Class to represent a ground height grid for ground proximity checks
 *
 */
class GroundGrid {
public:
  GroundGrid() = default;

  /**
   * @brief Constructor for GroundGrid
   *
   * @param range Maximum range of the grid
   * @param angle Angular resolution
   * @param radius Base radial bin size
   * @param start_augmentation Distance at which augmentation starts
   * @param radius_augmentation Bin size increase per bin
   * @param fov Field of view
   */
  GroundGrid(const double range, const double angle, const double radius,
             const double start_augmentation, const double radius_augmentation, const double fov);

  /**
   * @brief Get the ground height at a specific (x, y) location
   * @param x X coordinate
   * @param y Y coordinate
   * @return Ground height (float), or NaN if no data
   */
  float get_ground_height(const float x, const float y) const;

  /**
   * @brief Set the ground height at a specific (x, y) location
   * @param x X coordinate
   * @param y Y coordinate
   * @param height Ground height to set
   */
  void set_ground_height(const float x, const float y, const float height);

  /**
   * @brief Reset the ground grid to initial state
   */
  void reset_grid();

private:
  double range_;                          // Maximum range of the grid
  GridGeometry grid_geometry_;            // Grid geometry struct for grid calculations
  std::vector<std::vector<float>> grid_;  // 2D grid to hold ground heights, initialized to NaN
};
