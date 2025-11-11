#pragma once

#include <cmath>

/**
 * @struct GridGeometry
 *
 * @brief Structure to hold grid geometry parameters and provide utility functions
 *
 */
struct GridGeometry {
  double angle;                // Angular resolution (°)
  double radius;               // Base radial bin size (m)
  double start_augmentation;   // Distance at which augmentation starts
  double radius_augmentation;  // Bin size increase per bin (m)
  double fov;                  // Field of view (°)

  /**
   * @brief Constructor to initialize grid geometry parameters
   *
   * @param angle_ Angular resolution
   * @param radius_ Base radial bin size
   * @param start_aug_ Distance at which augmentation starts
   * @param radius_aug_ Bin size increase per bin
   * @param fov_ Field of view
   */
  GridGeometry(double angle_, double radius_, double start_aug_, double radius_aug_, double fov_);

  /**
   * @brief Compute slice index for a given (x, y)
   * @param x X coordinate
   * @param y Y coordinate
   * @return Slice index
   */
  int get_slice_index(double x, double y) const;

  /**
   * @brief Compute bin index for a given (x, y)
   * @param x X coordinate
   * @param y Y coordinate
   * @return Bin index
   */
  int get_bin_index(double x, double y) const;

  /**
   * @brief Get the number of slices in the grid
   */
  int get_num_slices() const;

  /**
   * @brief Get the number of bins for a given range
   * @param range Distance from origin
   * @return Number of bins
   */
  int get_num_bins(double range) const;

  /**
   * @brief Get the number of constant bins before augmentation starts
   * @return Number of constant bins
   */
  int get_num_constant_bins() const;
};