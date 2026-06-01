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

  if (n < 2) {
    sections_.push_back(
        {0, n - 1, 0.0, 0, config_.longitudinal_acceleration_, config_.lateral_acceleration_});
    return;
  }

  const double curvature_jump_threshold = curvature_peak_threshold_;
  // reuse your param OR define new one

  int start = 0;

  double running_mean = curvatures[0];
  int count = 1;

  for (int i = 1; i < n; ++i) {
    double prev_mean = running_mean / count;
    double cur = curvatures[i];

    // update running mean
    running_mean += cur;
    count++;

    double new_mean = running_mean / count;

    double jump = std::abs(cur - prev_mean);

    bool split = false;

    // ignore near-zero curvature noise
    if (std::abs(cur - curvatures[i - 1]) > curvature_jump_threshold) {
      split = true;
    }

    // also split if curvature regime changes significantly
    if (jump > curvature_jump_threshold) {
      split = true;
    }

    // avoid tiny sections
    if (split && (i - start) > min_section_spacing_) {
      sections_.push_back({start, i - 1, 0.0, 0, config_.longitudinal_acceleration_,
                           config_.lateral_acceleration_});

      start = i;
      running_mean = cur;
      count = 1;
    }
  }

  // last section
  sections_.push_back(
      {start, n - 1, 0.0, 0, config_.longitudinal_acceleration_, config_.lateral_acceleration_});
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
    int sec_idx = find_section(i);
    double lat_acc =
        (sec_idx >= 0) ? sections_[sec_idx].current_lat_acc : config_.lateral_acceleration_;
    double velocity = std::sqrt(lat_acc / std::abs(curvatures[i]));
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

    int sec_idx = find_section(i);
    double lateral_acc =
        (sec_idx >= 0) ? sections_[sec_idx].current_lat_acc : config_.lateral_acceleration_;
    double longitudinal_acc =
        (sec_idx >= 0) ? sections_[sec_idx].current_long_acc : config_.longitudinal_acceleration_;

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

    int sec_idx = find_section(i);
    double lateral_acc =
        (sec_idx >= 0) ? sections_[sec_idx].current_lat_acc : config_.lateral_acceleration_;

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

  std::vector<double> curvatures(path_size, 0.0);
  for (int i = 1; i < path_size - 1; ++i) {
    curvatures[i] = find_curvature(final_path[i - 1], final_path[i], final_path[i + 1]);
  }

  // Compute sections on first call or when path size changes
  if (sections_.empty() || static_cast<int>(sections_.back().end_idx) != path_size - 1) {
    compute_sections(curvatures);
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

  // Compute sections on the real path once, before tripling
  if ((sections_.empty() || static_cast<int>(sections_.back().end_idx) != path_size - 1)) {
    std::vector<double> curvatures(path_size, 0.0);
    for (int i = 1; i < path_size - 1; ++i) {
      curvatures[i] = find_curvature(final_path[i - 1], final_path[i], final_path[i + 1]);
    }
    compute_sections(curvatures);
  }

  // Save sections computed on the real path — set_velocity on the tripled
  // path would overwrite them with wrong indices
  std::vector<Section> saved_sections = sections_;

  std::vector<PathPoint> triple_path;
  triple_path.reserve(3 * path_size);
  for (int lap = 0; lap < 3; ++lap) {
    for (int i = 0; i < path_size; ++i) {
      triple_path.push_back(final_path[i]);
    }
  }

  // Temporarily expand section indices for the tripled path so the velocity
  // passes read the correct per-section limits
  // Middle lap offset is path_size, so shift each section by that for the
  // passes — actually simpler: just suppress section recomputation inside
  // set_velocity and let it use whatever sections_ contains.
  // Since triple_path indices [path_size, 2*path_size-1] map to the same
  // corners as [0, path_size-1], we replicate sections_ three times.
  sections_.clear();
  for (int lap = 0; lap < 3; ++lap) {
    for (const auto &sec : saved_sections) {
      sections_.push_back({sec.start_idx + lap * path_size, sec.end_idx + lap * path_size,
                           sec.mean_error, sec.sample_count, sec.current_long_acc,
                           sec.current_lat_acc});
    }
  }

  // Run velocity passes on tripled path — set_velocity will skip recomputation
  // because sections_.back().end_idx == 3*path_size-1 == triple_path.size()-1
  set_velocity(triple_path);

  // Extract the middle lap
  int offset = path_size;
  for (int i = 0; i < path_size; ++i) {
    final_path[i].ideal_velocity = triple_path[offset + i].ideal_velocity;
  }

  // Restore real-path sections
  sections_ = saved_sections;
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
    int sec_idx = find_section(index);
    double lateral_acc =
        (sec_idx >= 0) ? sections_[sec_idx].current_lat_acc : config_.lateral_acceleration_;

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

  while (index < (path_size - path_size / 4)) {
    final_path[index].ideal_velocity = 0.0;
    ++index;
  }
}

void VelocityPlanning::change_section_limits(int section_idx, double delta_long, double delta_lat) {
  if (section_idx < 0 || section_idx >= static_cast<int>(sections_.size())) {
    return;
  }
  sections_[section_idx].current_long_acc += delta_long;
  sections_[section_idx].current_lat_acc += delta_lat;
}

void VelocityPlanning::adapt_limits(Pose &pose, std::vector<PathPoint> &path, bool is_closed) {
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

  double delta;

  double scale_pos = 1.35;  // tune: 1.2–1.6

  if (mean <= 0.2) {
    double t = mean / 0.2;
    delta = scale_pos * (0.4 + t * (0.2 - 0.4));
  } else if (mean <= 0.55) {
    double t = (mean - 0.2) / 0.35;
    delta = scale_pos * (0.2 + t * (0.05 - 0.2));
  } else if (mean <= 0.75) {
    double t = (mean - 0.55) / 0.2;
    delta = 0.05 + t * (-0.5 - 0.05);
  } else if (mean <= 1.0) {
    double t = (mean - 0.75) / 0.25;
    delta = -0.5 + t * (-1.2 + 0.5);
  } else {
    delta = -1.2;
  }

  change_section_limits(sec_idx, delta, delta);

  // Reset accumulator for the next window
  sec.mean_error = 0.0;
  sec.sample_count = 0;

  if (is_closed) {
    trackdrive_velocity(path);
  } else {
    set_velocity(path);
  }
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