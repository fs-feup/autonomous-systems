#pragma once

#include <vector>

#include "utils/precone.hpp"

class TrackFilter {
public:
  TrackFilter(double min_distance, double max_distance, int n_cones);

  /**
   * @brief Filters the cones based on the track criteria.
   *
   * This method filters out cones that are not within the specified distance range
   * and do not meet the minimum number of cones criteria.
   *
   * @param cones A vector of cones to be filtered.
   */
  void filter(std::vector<PreCone>& cones);

private:
  double min_distance_;  ///< Minimum distance from the track center.
  double max_distance_;  ///< Maximum distance from the track center.
  int n_cones_;  ///< Minimum number of cones required in proxmity for a cone to be part of the
                 ///< track.
};
