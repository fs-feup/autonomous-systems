#pragma once

namespace common_lib::structures {

/**
 * @brief Represents a contiguous section of the path between curvature peaks/apexes.
 */
struct Section {
  int start_idx;
  int end_idx;
  double mean_error;
  int sample_count;
  double current_long_acc;
  double current_lat_acc;
  double max_error;
  double min_error;
  bool is_corner;
};

}  // namespace common_lib::structures
