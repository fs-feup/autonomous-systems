#include "planning/velocity_planning.hpp"

double VelocityPlanning::find_curvature(const PathPoint &p1, const PathPoint &p2,
                                        const PathPoint &p3) {
  double a = std::hypot(p2.position.x - p1.position.x, p2.position.y - p1.position.y);
  double b = std::hypot(p3.position.x - p2.position.x, p3.position.y - p2.position.y);
  double c = std::hypot(p3.position.x - p1.position.x, p3.position.y - p1.position.y);

  double area = 0.5 * std::abs(p1.position.x * (p2.position.y - p3.position.y) +
                               p2.position.x * (p3.position.y - p1.position.y) +
                               p3.position.x * (p1.position.y - p2.position.y));

  if (a * b * c < epsilon) {
    return 0.0;
  }

  return 4 * area / (a * b * c);
}

void VelocityPlanning::compute_sections(const std::vector<double> &curvatures) {
  sections_.clear();
  int n = static_cast<int>(curvatures.size());

  // Find local curvature maxima above threshold 
  // A point is a peak if it is strictly greater than all neighbours within
  // min_section_spacing_ and above curvature_peak_threshold_.
  std::vector<int> peaks;
  for (int i = 1; i < n - 1; ++i) {
    if (curvatures[i] < curvature_peak_threshold_) {
      continue;
    }

    bool is_peak = true;
    int half = min_section_spacing_ / 2;
    for (int k = std::max(0, i - half); k <= std::min(n - 1, i + half); ++k) {
      if (k != i && curvatures[k] >= curvatures[i]) {
        is_peak = false;
        break;
      }
    }
    if (is_peak) {
      peaks.push_back(i);
    }
  }

  // Merge peaks that are too close together 
  // Keep only the highest peak within any window of min_section_spacing_ points.
  std::vector<int> merged_peaks;
  for (int i = 0; i < static_cast<int>(peaks.size()); ++i) {
    if (!merged_peaks.empty() && peaks[i] - merged_peaks.back() < min_section_spacing_) {
      // Replace with whichever has higher curvature
      if (curvatures[peaks[i]] > curvatures[merged_peaks.back()]) {
        merged_peaks.back() = peaks[i];
      }
    } else {
      merged_peaks.push_back(peaks[i]);
    }
  }

  // Build sections between consecutive peaks  
  // Each section spans from one peak to the next (inclusive on both ends).
  // If no peaks were found, the whole path is one section.
  if (merged_peaks.empty()) {
    sections_.push_back({0, n - 1, 0.0, 0});
    return;
  }

  // Section before first peak: index 0 → first peak
  sections_.push_back({0, merged_peaks[0], 0.0, 0});

  // Sections between consecutive peaks
  for (int i = 0; i + 1 < static_cast<int>(merged_peaks.size()); ++i) {
    sections_.push_back({merged_peaks[i], merged_peaks[i + 1], 0.0, 0});
  }

  // Section after last peak: last peak → end
  sections_.push_back({merged_peaks.back(), n - 1, 0.0, 0});
}


int VelocityPlanning::find_section(int point_idx) const {
  for (int s = 0; s < static_cast<int>(sections_.size()); ++s) {
    if (point_idx >= sections_[s].start_idx && point_idx <= sections_[s].end_idx) {
      return s;
    }
  }
  return -1;
}

void VelocityPlanning::point_speed(const std::vector<double> &curvatures,
                                   std::vector<double> &velocities) {
  for (int i = 0; i < (int)curvatures.size(); i++) {
    if (std::abs(curvatures[i]) < epsilon) {
      velocities.push_back(config_.desired_velocity_);
      continue;
    }
    double velocity = std::sqrt(max_lateral_acceleration_[i] / std::abs(curvatures[i]));
    velocities.push_back(std::min(velocity, config_.desired_velocity_));
  }
  velocities.back() = config_.minimum_velocity_;
}

void VelocityPlanning::acceleration_limiter(const std::vector<PathPoint> &points,
                                            std::vector<double> &velocities,
                                            const std::vector<double> &curvatures) {
  velocities[0] = config_.minimum_velocity_;
  for (int i = 1; i < (int)points.size(); i++) {
    double dx = points[i].position.x - points[i - 1].position.x;
    double dy = points[i].position.y - points[i - 1].position.y;
    double d = std::sqrt(dx * dx + dy * dy);

    double lateral_acc = max_lateral_acceleration_[i];
    double longitudinal_acc = max_longitudinal_acceleration_[i];

    double ay =
        std::min(velocities[i - 1] * velocities[i - 1] * std::abs(curvatures[i - 1]), lateral_acc);
    double ax_max =
        longitudinal_acc * std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));
    ax_max = std::min(ax_max, longitudinal_acc);

    double max_velocity =
        std::sqrt(std::max(0.0, velocities[i - 1] * velocities[i - 1] + 2 * ax_max * d));
    velocities[i] = std::min(velocities[i], max_velocity);
  }
}

void VelocityPlanning::braking_limiter(std::vector<PathPoint> &points,
                                       std::vector<double> &velocities,
                                       const std::vector<double> &curvatures) {
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; i--) {
    int j = i + 1;
    double distance = std::hypot(points[j].position.x - points[i].position.x,
                                 points[j].position.y - points[i].position.y);

    double lateral_acc = max_lateral_acceleration_[i];

    double ay = std::min(velocities[j] * velocities[j] * std::abs(curvatures[j]), lateral_acc);

    double ax_brake = config_.braking_acceleration_ *
                      std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));

    double max_speed =
        std::sqrt(std::max(0.0, velocities[j] * velocities[j] + 2 * ax_brake * distance));
    max_speed = std::min(max_speed, config_.desired_velocity_);
    velocities[i] = std::min(max_speed, velocities[i]);
  }
}


void VelocityPlanning::set_velocity(std::vector<PathPoint> &final_path) {
  int path_size = static_cast<int>(final_path.size());

  if (!config_.use_velocity_planning_ || path_size <= 3) {
    for (auto &p : final_path) {
      p.ideal_velocity = config_.minimum_velocity_;
    }
    return;
  }

  // Grow per-node limit vectors if the path grew
  while (static_cast<int>(max_longitudinal_acceleration_.size()) < path_size) {
    max_longitudinal_acceleration_.push_back(config_.longitudinal_acceleration_);
    max_lateral_acceleration_.push_back(config_.lateral_acceleration_);
  }

  // Curvature at every point
  std::vector<double> curvatures(path_size, 0.0);
  for (int i = 1; i < path_size - 1; ++i) {
    curvatures[i] = find_curvature(final_path[i - 1], final_path[i], final_path[i + 1]);
  }

  // Velocity passes
  std::vector<double> velocities;
  point_speed(curvatures, velocities);
  acceleration_limiter(final_path, velocities, curvatures);
  braking_limiter(final_path, velocities, curvatures);

  for (int i = 0; i < path_size; ++i) {
    velocities[i] = std::max(velocities[i], config_.minimum_velocity_);
    velocities[i] = std::min(velocities[i], config_.desired_velocity_);
    final_path[i].ideal_velocity = velocities[i];
  }
}

void VelocityPlanning::trackdrive_velocity(std::vector<PathPoint> &final_path) {
  int path_size = static_cast<int>(final_path.size());
  if (!config_.use_velocity_planning_ || path_size <= 3) {
    for (auto &p : final_path) {
      p.ideal_velocity = config_.minimum_velocity_;
    }
    return;
  }

  // Triple the path to allow the forward/backward passes to see the wrap-around
  std::vector<PathPoint> triple_path;
  triple_path.reserve(3 * path_size);
  for (int lap = 0; lap < 3; ++lap) {
    for (int i = 0; i < path_size; ++i) {
      triple_path.push_back(final_path[i]);
    }
  }

  set_velocity(triple_path);

  // Extract the middle lap
  int offset = path_size;
  for (int i = 0; i < path_size; ++i) {
    final_path[i].ideal_velocity = triple_path[offset + i].ideal_velocity;
  }

  // Sections were computed on the tripled path; recompute on the actual path length
  // so that section indices are valid for adapt_limits() calls later.
  std::vector<double> curvatures(path_size, 0.0);
  for (int i = 1; i < path_size - 1; ++i) {
    curvatures[i] = find_curvature(final_path[i - 1], final_path[i], final_path[i + 1]);
  }

  if (sections_.empty()) {
    compute_sections(curvatures);
  }
}

void VelocityPlanning::stop(std::vector<PathPoint> &final_path, double braking_distance) {
  int path_size = final_path.size();
  if (path_size <= 3) {
    for (auto &p : final_path) {
      p.ideal_velocity = config_.minimum_velocity_;
    }
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough path points to do velocity profile.");
    return;
  }

  double dist = 0.0;
  int index = 1;
  while (dist < braking_distance && index < path_size) {
    dist += final_path[index].position.euclidean_distance(final_path[index - 1].position);
    ++index;
  }

  while (index < static_cast<int>(path_size) - 1 && final_path[index].ideal_velocity > 0.0) {
    int j = index + 1;
    double d = final_path[j].position.euclidean_distance(final_path[index].position);
    double vi = final_path[index].ideal_velocity;
    double lateral_acc = max_lateral_acceleration_[index];

    double ay = std::min(vi * vi *
                             std::abs(find_curvature(final_path[std::max(index - 1, 0)],
                                                     final_path[index], final_path[j])),
                         lateral_acc);

    double ax_available = config_.braking_acceleration_ *
                          std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));

    double vj = std::sqrt(std::max(0.0, vi * vi - 2.0 * ax_available * d));
    vj = std::max(vj, 0.0);

    final_path[j].ideal_velocity = std::min(final_path[j].ideal_velocity, vj);
    ++index;
  }

  while (index < path_size) {
    final_path[index].ideal_velocity = 0.0;
    ++index;
  }
}

void VelocityPlanning::change_section_limits(int section_idx, double longitudinal_acc,
                                             double lateral_acc) {
  if (section_idx < 0 || section_idx >= static_cast<int>(sections_.size())) {
    return;
  }
  const Section &sec = sections_[section_idx];
  for (int i = sec.start_idx; i <= sec.end_idx; ++i) {
    change_limits(i, longitudinal_acc, lateral_acc);
  }
}

void VelocityPlanning::change_limits(int index, double longitudinal_acc, double lateral_acc) {
  if (index < static_cast<int>(max_longitudinal_acceleration_.size())) {
    max_longitudinal_acceleration_[index] += longitudinal_acc;
    max_lateral_acceleration_[index] += lateral_acc;
  }
}

void VelocityPlanning::change_all_limits(double longitudinal_acc, double lateral_acc) {
  for (size_t i = 0; i < max_longitudinal_acceleration_.size(); ++i) {
    change_limits(i, longitudinal_acc, lateral_acc);
  }
}

void VelocityPlanning::adapt_limits(Pose &pose, std::vector<PathPoint> &path) {
  // Find the closest path segment and the cross-track error 
  size_t point_idx = 0;
  double error = get_pose_error(pose, path, point_idx);
  if (error < 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Cannot adapt limits, invalid path.");
    return;
  }

  // Identify which section the vehicle is in 
  int sec_idx = find_section(static_cast<int>(point_idx));
  if (sec_idx < 0) {
    return;  // No sections computed yet; silently skip
  }
  Section &sec = sections_[sec_idx];

  // Accumulate error into the rolling mean 
  // Online (Welford-style) mean update: mean += (x - mean) / n
  ++sec.sample_count;
  sec.mean_error += (error - sec.mean_error) / static_cast<double>(sec.sample_count);

  // Apply adjustment once we have enough samples 
  if (sec.sample_count < section_adapt_samples_) {
    return;
  }

  const double mean = sec.mean_error;

  // Same error bands as before, but applied to the whole section at once.
  // Negative deltas tighten limits (slower, safer); positive deltas relax them.
  if (mean > 1.0) {
    // Large error: the car is struggling badly — reduce limits significantly
    change_section_limits(sec_idx, -1.0, -1.0);
  } else if (mean > 0.5) {
    // Moderate error: slight tightening
    change_section_limits(sec_idx, -0.2, -0.2);
  } else if (mean > 0.2) {
    // Small error: car is coping well — gently relax limits
    change_section_limits(sec_idx, 0.5, 0.5);
  } else {
    // Very small error: car is very comfortable — relax more
    change_section_limits(sec_idx, 1.0, 1.0);
  }

  // Reset accumulator for the next window
  sec.mean_error = 0.0;
  sec.sample_count = 0;
}

double VelocityPlanning::get_pose_error(const Pose &pose, const std::vector<PathPoint> &path,
                                        size_t &best_index) {
  if (path.size() < 2) {
    return -1.0;
  }

  double best_dist_sq = std::numeric_limits<double>::max();

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const auto &before = path[i].position;
    const auto &after = path[i + 1].position;

    double x = after.x - before.x;
    double y = after.y - before.y;
    double len_sq = x * x + y * y;

    if (len_sq < 1e-9) {
      continue;
    }

    double px = pose.position.x - before.x;
    double py = pose.position.y - before.y;
    double t = std::clamp((px * x + py * y) / len_sq, 0.0, 1.0);

    double proj_x = before.x + t * x;
    double proj_y = before.y + t * y;

    double dx = pose.position.x - proj_x;
    double dy = pose.position.y - proj_y;
    double dist_sq = dx * dx + dy * dy;

    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_index = i;
    }
  }

  if (best_dist_sq == std::numeric_limits<double>::max()) {
    return -1.0;
  }

  return std::sqrt(best_dist_sq);
}