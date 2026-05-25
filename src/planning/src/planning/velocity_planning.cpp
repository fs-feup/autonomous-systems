#include "planning/velocity_planning.hpp"

double VelocityPlanning::find_curvature(const PathPoint &p1, const PathPoint &p2,
                                        const PathPoint &p3) {
  // lengths of the sides of the triangle formed by the three points
  double a = std::hypot(p2.position.x - p1.position.x, p2.position.y - p1.position.y);
  double b = std::hypot(p3.position.x - p2.position.x, p3.position.y - p2.position.y);
  double c = std::hypot(p3.position.x - p1.position.x, p3.position.y - p1.position.y);

  // area of the triangle using the determinant method
  double area = 0.5 * std::abs(p1.position.x * (p2.position.y - p3.position.y) +
                               p2.position.x * (p3.position.y - p1.position.y) +
                               p3.position.x * (p1.position.y - p2.position.y));

  // To avoid division by zero in case of near duplicate points
  if (a * b * c < epsilon) {
    return 0.0;
  }

  /* The Menger curvature is given by the formula: K = 4A / (abc),
  where A is the triangle area and a,b,c are the side lengths*/
  return 4 * area / (a * b * c);
}

void VelocityPlanning::point_speed(const std::vector<double> &curvatures,
                                   std::vector<double> &velocities) {
  for (int i = 0; i < (int)curvatures.size(); i++) {
    // This is a straight line, there is no curvature limit on the velocity
    if (std::abs(curvatures[i]) < epsilon) {
      velocities.push_back(config_.desired_velocity_);
      continue;
    }
    // Per-point lateral acceleration limit
    double velocity = std::sqrt(max_lateral_acceleration_[i] / std::abs(curvatures[i]));
    velocities.push_back(std::min(velocity, config_.desired_velocity_));
  }
  // The last point is always the minimum velocity for safety
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

    // lateral acceleration at previous point: v(i-1)^2 * curvature
    double ay =
        std::min(velocities[i - 1] * velocities[i - 1] * std::abs(curvatures[i - 1]), lateral_acc);
    // Friction ellipse: (ax/ax_max)^2 + (ay/ay_max)^2 = 1
    double ax_max =
        longitudinal_acc * std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));

    // Cap by acceleration limit
    ax_max = std::min(ax_max, longitudinal_acc);

    // v_i^2 = v_(i-1)^2 + 2 * a_x_available * d
    double max_velocity =
        std::sqrt(std::max(0.0, velocities[i - 1] * velocities[i - 1] + 2 * ax_max * d));
    velocities[i] = std::min(velocities[i], max_velocity);
  }
}

void VelocityPlanning::braking_limiter(std::vector<PathPoint> &points,
                                       std::vector<double> &velocities,
                                       const std::vector<double> &curvatures) {
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; i--) {
    // Calculate segment distance
    int j = i + 1;
    double distance = std::hypot(points[j].position.x - points[i].position.x,
                                 points[j].position.y - points[i].position.y);

    double lateral_acc = max_lateral_acceleration_[i];

    // Lateral acceleration at the next point: a = v(j)^2 * curvature
    // Clamped to lateral_acceleration_ to avoid ay exceeding the lateral acceleration limit
    double ay = std::min(velocities[j] * velocities[j] * std::abs(curvatures[j]), lateral_acc);

    // Friction ellipse: remaining longitudinal braking

    double ax_brake = config_.braking_acceleration_ *
                      std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));

    // Correct kinematic speed calculation
    // v_f² = v_i² + 2ad
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

  if (static_cast<int>(max_longitudinal_acceleration_.size()) < path_size) {
    while (static_cast<int>(max_longitudinal_acceleration_.size()) < path_size) {
      max_longitudinal_acceleration_.push_back(config_.longitudinal_acceleration_);
      max_lateral_acceleration_.push_back(config_.lateral_acceleration_);
    }
  }

  std::vector<double> curvatures(path_size, 0.0);
  for (int i = 1; i < path_size - 1; ++i) {
    curvatures[i] = find_curvature(final_path[i - 1], final_path[i], final_path[i + 1]);
  }

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

  // Triple the path
  std::vector<PathPoint> triple_path;
  triple_path.reserve(3 * path_size);
  for (int lap = 0; lap < 3; ++lap) {
    for (int i = 0; i < path_size; ++i) {
      triple_path.push_back(final_path[i]);
    }
  }

  set_velocity(triple_path);

  // Extract middle path velocities ----
  int offset = path_size;  // middle lap start
  for (int i = 0; i < path_size; ++i) {
    final_path[i].ideal_velocity = triple_path[offset + i].ideal_velocity;
  }
}

void VelocityPlanning::stop(std::vector<PathPoint> &final_path, double braking_distance) {
  int path_size = final_path.size();
  if (path_size <= 3) {
    for (auto &p : final_path) {
      p.ideal_velocity = config_.minimum_velocity_;
    }
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough path point to do velocity profile.");
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

    // Segment distance
    double d = final_path[j].position.euclidean_distance(final_path[index].position);

    double vi = final_path[index].ideal_velocity;

    double lateral_acc = max_lateral_acceleration_[index];

    // Lateral acceleration at current point
    // Clamped to lateral_acceleration_ to avoid ay exceeding the lateral grip limit
    double ay = std::min(vi * vi *
                             std::abs(find_curvature(final_path[std::max(index - 1, 0)],
                                                     final_path[index], final_path[j])),
                         lateral_acc);

    // Friction ellipse: remaining longitudinal braking
    double ax_available = config_.braking_acceleration_ *
                          std::sqrt(std::max(0.0, 1.0 - std::pow(ay / lateral_acc, 2)));

    // Forward braking kinematics: v_j^2 = v_i^2 + 2 * a * d
    double vj = std::sqrt(std::max(0.0, vi * vi - 2.0 * ax_available * d));
    vj = std::max(vj, 0.0);

    final_path[j].ideal_velocity = std::min(final_path[j].ideal_velocity, vj);
    ++index;
  }

  // After the car stop the rest of the points should have 0.0 speed
  while (index < path_size) {
    final_path[index].ideal_velocity = 0.0;
    ++index;
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
  size_t index = 0;
  double error = get_pose_error(pose, path, index);
  if (error < 0) { 
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Cannot adapt limits, invalid path."); 
    return;
  }

  //change to values that make sense:
  if (error > 1.0) { 
    change_limits(index, -1.0, -1.0);
    change_limits(index+1, -1.0, -1.0);
    //change_all_limits(-0.1, -0.1);
  } else if (error > 0.5) { 
    change_limits(index, 0.2, 0.2);
    change_limits(index+1, 0.2, 0.2);
    //change_all_limits(-0.02, -0.02);
  } else if (error > 0.2) { 
    change_limits(index, 1.0, 1.0);
    change_limits(index+1, 1.0, 1.0);
    //change_all_limits(0.02, 0.02);
  } else {
    change_limits(index, 1.5, 1.5);
    change_limits(index+1, 1.5, 1.5);
    //change_all_limits(0.01, 0.01);
  }

  return; 
}