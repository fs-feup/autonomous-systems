#include "planning/velocity_planning.hpp"

double longitudinal_acceleration = 8.0;
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
  for (const auto &k : curvatures) {
    // This is a straight line, there is no curvature limit on the velocity
    if (std::abs(k) < epsilon) {
      velocities.push_back(config_.desired_velocity_);
      continue;
    }

    double velocity = std::sqrt(config_.lateral_acceleration_ / std::abs(k));
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

    // lateral acceleration at previous point: v(i-1)^2 * curvature
    double ay = std::min(velocities[i - 1] * velocities[i - 1] * std::abs(curvatures[i - 1]),
                         config_.lateral_acceleration_);
    double ax_max =
        std::sqrt(std::max(0.0, config_.acceleration_ * config_.acceleration_ - ay * ay));

    // Cap by acceleration limit
    ax_max = std::min(ax_max, longitudinal_acceleration);

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

    // Lateral acceleration at the next point: a = v(j)^2 * curvature
    // Clamped to lateral_acceleration_ to avoid ay exceeding the lateral acceleration limit
    double ay = std::min(velocities[j] * velocities[j] * std::abs(curvatures[j]),
                         config_.lateral_acceleration_);

    // Friction ellipse: remaining longitudinal braking
    double ax_brake =
        std::sqrt(std::max(0.0, config_.acceleration_ * config_.acceleration_ - ay * ay));

    // Cap by braking limit
    ax_brake = -(std::min(ax_brake, config_.braking_acceleration_));

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
  int path_size = static_cast<int>(final_path.size()) - 1;
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

  // Close loop explicitly
  final_path.back().ideal_velocity = final_path.front().ideal_velocity;
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

    // Lateral acceleration at current point
    // Clamped to lateral_acceleration_ to avoid ay exceeding the lateral grip limit
    double ay = std::min(vi * vi *
                             std::abs(find_curvature(final_path[std::max(index - 1, 0)],
                                                     final_path[index], final_path[j])),
                         config_.lateral_acceleration_);

    // Friction ellipse: remaining longitudinal braking
    double ax_available =
        std::sqrt(std::max(0.0, config_.acceleration_ * config_.acceleration_ - ay * ay));

    // Cap with maximum braking capability
    ax_available = -(std::min(ax_available, -config_.braking_acceleration_));

    // Forward braking kinematics: v_j^2 = v_i^2 + 2 * a * d
    double vj = std::sqrt(std::max(0.0, vi * vi + 2.0 * ax_available * d));
    vj = std::max(vj, 0.0);

    final_path[j].ideal_velocity = std::min(final_path[j].ideal_velocity, vj);
    ++index;
  }

  // After the car stop the rest of the points should have 0.0 speed
  while (index < path_size) {
    final_path[index].ideal_velocity = 0.0;
    ++index;
  }

  for (size_t i = 1; i < path_size; ++i) {
    if (final_path[i - 1].ideal_velocity > final_path[i].ideal_velocity) {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "ERROR");
    }
    if (final_path[i].ideal_velocity == 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("planning"), "Car stops at index %zu, all points %zu", i,
                   path_size);
      break;
    }
  }
}