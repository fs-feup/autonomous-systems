#include "planning/velocity_planning.hpp"

double VelocityPlanning::find_circle_center(const PathPoint &point1, const PathPoint &point2,
                                            const PathPoint &point3) {
  double x1 = point1.position.x;
  double y1 = point1.position.y;
  double x2 = point2.position.x;
  double y2 = point2.position.y;
  double x3 = point3.position.x;
  double y3 = point3.position.y;

  PathPoint mid1 = PathPoint((x1 + x2) / 2, (y1 + y2) / 2, 0);
  PathPoint mid2 = PathPoint((x2 + x3) / 2, (y2 + y3) / 2, 0);

  double slope1 = MAXFLOAT;
  double slope2 = MAXFLOAT;
  double slope1_perpendicular = 10'000;
  double slope2_perpendicular = 10'000;
  if (x2 != x1) {
    slope1 = (y2 - y1) / (x2 - x1);
  }

  if (x3 != x2) {
    slope2 = (y3 - y2) / (x3 - x2);
  }

  if (slope1 != 0) {
    slope1_perpendicular = -1 / slope1;
  }

  if (slope2 != 0) {
    slope2_perpendicular = -1 / slope2;
  }

  if (slope1_perpendicular == slope2_perpendicular) {
    return 10'000;
  }

  double center_x = (slope1_perpendicular * mid1.position.x -
                     slope2_perpendicular * mid2.position.x + mid2.position.y - mid1.position.y) /
                    (slope1_perpendicular - slope2_perpendicular);
  double center_y = slope1_perpendicular * (center_x - mid1.position.x) + mid1.position.y;
  double radius = std::sqrt(std::pow(center_x - x2, 2) + std::pow(center_y - y2, 2));
  return radius;
}

void VelocityPlanning::point_speed(const std::vector<double> &radiuses,
                                   std::vector<double> &velocities) {
  for (const auto &radius : radiuses) {
    double velocity = std::sqrt(abs(config_.normal_acceleration_ * radius));
    velocities.push_back(std::max(velocity, config_.minimum_velocity_));
  }
  velocities.back() = config_.minimum_velocity_;

  return;
}

void VelocityPlanning::accelaration_limiter(const std::vector<PathPoint> &points,
                                   std::vector<double> &velocities) {
  velocities[0] = config_.minimum_velocity_;
  for (int i = 1; i < (int)points.size(); i++) {
    // distance to previous point
    double dx = points[i].position.x - points[i - 1].position.x;
    double dy = points[i].position.y - points[i - 1].position.y;
    double d = std::sqrt(dx * dx + dy * dy);

    // v_i^2 = v_(i-1)^2 + 2 * a_acc * d
    double max_velocity = std::sqrt(
        std::max(0.0, velocities[i - 1] * velocities[i - 1] + 2.0 * config_.acceleration_ * d));

    // cap the curvature-based velocity by longitudinal acceleration
    velocities[i] = std::min(velocities[i], max_velocity);
  }
}

void VelocityPlanning::braking_limiter(std::vector<PathPoint> &points,
                                     std::vector<double> &velocities) {
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; i--) {
    double distance = 0;
    double max_speed = velocities[i];

    // Calculate segment distance
    int j = i + 1;
    distance = std::sqrt(std::pow(points[j].position.x - points[j - 1].position.x, 2) +
                         std::pow(points[j].position.y - points[j - 1].position.y, 2));

    // Correct kinematic speed calculation
    // v_f² = v_i² + 2ad
    double max_terminal_speed = std::sqrt(
        std::max(0.0, std::pow(velocities[j], 2) - 2 * config_.braking_acceleration_ * distance));

    max_speed = std::min(max_speed, max_terminal_speed);
    max_speed = std::min(max_speed, this->config_.desired_velocity_);

    velocities[i] = max_speed;
  }
}

// Main function to set the velocity of the car
void VelocityPlanning::set_velocity(std::vector<PathPoint> &final_path) {
  if ((config_.use_velocity_planning_) && (final_path.size() > 2)) {
    std::vector<double> radiuses;
    radiuses.push_back(0);
    for (int i = 1; i < static_cast<int>(final_path.size()) - 1; i++) {
      radiuses.push_back(find_circle_center(final_path[i - 1], final_path[i], final_path[i + 1]));
    }
    radiuses[0] = radiuses[1];
    radiuses.push_back(radiuses.back());

    std::vector<double> velocities;
    point_speed(radiuses, velocities);
    accelaration_limiter(final_path, velocities);
    braking_limiter(final_path, velocities);

    for (int i = 0; i < static_cast<int>(final_path.size()); i++) {
      final_path[i].ideal_velocity = velocities[i];
    }
  }

  else {
    for (auto &path_point : final_path) {
      path_point.ideal_velocity = config_.minimum_velocity_;
    }
  }
}

void VelocityPlanning::trackdrive_velocity(std::vector<PathPoint> &final_path) {
  if ((config_.use_velocity_planning_) && (final_path.size() > 2)) {
    // Calculate curvature radiuses for all points
    std::vector<double> radiuses;
    radiuses.push_back(0);
    for (int i = 1; i < static_cast<int>(final_path.size()) - 1; i++) {
      radiuses.push_back(find_circle_center(final_path[i - 1], final_path[i], final_path[i + 1]));
    }
    radiuses[0] = radiuses[1];
    radiuses.push_back(radiuses.back());

    // Calculate curvature-limited velocities
    std::vector<double> velocities;
    point_speed(radiuses, velocities);

    velocities.back() = velocities[0]; // Closed loop consistency
    
    // Apply acceleration limits
    accelaration_limiter(final_path, velocities);
    
    // Apply braking limits 
    braking_limiter(final_path, velocities);
    
    // Ensure start/end consistency after all passes
    velocities[0] = std::min(velocities[0], velocities.back());
    velocities.back() = velocities[0];

    // Apply velocities to path
    for (int i = 0; i < static_cast<int>(final_path.size()); i++) {
      final_path[i].ideal_velocity = velocities[i];
    }
  } else {
    for (auto &path_point : final_path) {
      path_point.ideal_velocity = config_.desired_velocity_;
    }
  }
}

void VelocityPlanning::stop(std::vector<PathPoint> &final_path) {
  int size = final_path.size();
  double dist = 0.0;
  for (int i = 0; i < size / 2; ++i) {
    dist += final_path[i].position.euclidean_distance(final_path[i + 1].position);
    if (dist > 10) {
      final_path[i].ideal_velocity = 0.0;
    }
  }
}
