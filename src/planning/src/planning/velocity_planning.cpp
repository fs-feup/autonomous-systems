#include "planning/velocity_planning.hpp"

double VelocityPlanning::find_circle_center(PathPoint &point1, PathPoint &point2,
                                            PathPoint &point3) {
  double x1 = point1.position.x;
  double y1 = point1.position.y;
  double x2 = point2.position.x;
  double y2 = point2.position.y;
  double x3 = point3.position.x;
  double y3 = point3.position.y;

  PathPoint mid1 = PathPoint((x1 + x2) / 2, (y1 + y2) / 2, 0);
  PathPoint mid2 = PathPoint((x2 + x3) / 2, (y2 + y3) / 2, 0);
  double slope1 = (x2 != x1) ? ((y2 - y1) / (x2 - x1)) : MAXFLOAT;
  double slope2 = (x3 != x2) ? ((y3 - y2) / (x3 - x2)) : MAXFLOAT;
  double slope1_perpendicular = -1 / slope1;
  double slope2_perpendicular = -1 / slope2;
  double center_x = (slope1_perpendicular * mid1.position.x -
                     slope2_perpendicular * mid2.position.x + mid2.position.y - mid1.position.y) /
                    (slope1_perpendicular - slope2_perpendicular);
  double center_y = slope1_perpendicular * (center_x - mid1.position.x) + mid1.position.y;
  double radius = sqrt(pow(center_x - x2, 2) + pow(center_y - y2, 2));
  return radius;
}

void VelocityPlanning::point_speed(std::vector<double> &radiuses, std::vector<double> &velocities) {
  for (int i = 0; i < static_cast<int>(radiuses.size()) - 1; i++) {
    double velocity = sqrt(abs(config_.braking_acceleration_ * radiuses[i]));
    velocities.push_back(std::max(velocity, config_.safety_speed_));
  }
  velocities.push_back(config_.safety_speed_);

  return;
}

void VelocityPlanning::speed_limiter(std::vector<PathPoint> &points,
                                     std::vector<double> &velocities) {
  for (int i = static_cast<int>(points.size()) - 2; i >= 0; i--) {
    double distance = 0;
    double max_speed = velocities[i];

    for (int j = i + 1; j < static_cast<int>(points.size()) - 1; j++) {
      // Calculate segment distance
      double segment_distance = sqrt(pow(points[j].position.x - points[j - 1].position.x, 2) +
                                     pow(points[j].position.y - points[j - 1].position.y, 2));
      distance += segment_distance;

      // Correct kinematic speed calculation
      // v_f² = v_i² + 2ad
      double max_terminal_speed = sqrt(
          std::max(0.0, pow(velocities[j], 2) - 2 * config_.braking_acceleration_ * distance)
      );

      max_speed = std::min(max_speed, max_terminal_speed);
    }
    velocities[i] = max_speed;
  }
}

// Main function to set the velocity of the car
void VelocityPlanning::set_velocity(std::vector<PathPoint> &final_path) {

  if (config_.use_velocity_planning_) {
    std::vector<double> radiuses;
    for (int i = 1; i < static_cast<int>(final_path.size()) - 1; i++) {
      radiuses.push_back(find_circle_center(final_path[i - 1], final_path[i], final_path[i + 1]));
    }
    radiuses[0] = radiuses[1];
    std::vector<double> velocities;
    point_speed(radiuses, velocities);
    speed_limiter(final_path, velocities);

    for (int i = 0; i < static_cast<int>(final_path.size()) - 1; i++) {
      final_path[i].ideal_velocity = velocities[i];
    }
  } 
  
  else {
    for (auto &path_point : final_path) {
      path_point.ideal_velocity = config_.safety_speed_;
    }
  }
  
}