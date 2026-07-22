#include "local_pather/alpha_beta.hpp"

double AlphaBeta::error_distance(custom_interfaces::msg::PathPointArray& msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) const {
  if (msg.pathpoint_array.size() < 2) {
    return 0.0;
  }

  const auto& p0 = msg.pathpoint_array[0];
  const auto& p1 = msg.pathpoint_array[1];

  const double sx = p1.x - p0.x;
  const double sy = p1.y - p0.y;
  const double vx = vehicle_state.x - p0.x;
  const double vy = vehicle_state.y - p0.y;

  const double seg_norm = std::sqrt(sx * sx + sy * sy);
  if (seg_norm < 1e-9) {
    return 0.0;
  }

  // Signed lateral distance to the segment's supporting line.
  // Positive: vehicle is to the left of segment direction p0 -> p1.
  return std::fabs((sx * vy - sy * vx) / seg_norm);
}

double AlphaBeta::convergence_distance(double error_distance) const {
  return 5.0 * std::log(error_distance + 1.0);
}

unsigned int AlphaBeta::number_of_points(custom_interfaces::msg::PathPointArray& msg, double convergence_distance) const {
  double distance_along_path = 0.0;
  unsigned int points = 1;

  for (std::size_t i = 1; i < msg.pathpoint_array.size(); ++i) {
    const auto& prev = msg.pathpoint_array[i - 1];
    const auto& current = msg.pathpoint_array[i];

    const double dx = current.x - prev.x;
    const double dy = current.y - prev.y;
    distance_along_path += std::sqrt(dx * dx + dy * dy);
    ++points;

    if (distance_along_path > convergence_distance) {
      return points;
    }
  }

  return points;
}

double AlphaBeta::phoenician_factor(double number_of_points) const {
  return std::pow(this->convergence_beta_, 1.0 / number_of_points);
}

void AlphaBeta::create_local_path(custom_interfaces::msg::PathPointArray& path_msg, const custom_interfaces::msg::VehicleStateVector& vehicle_state) {
  if (path_msg.pathpoint_array.empty()) {
    return;
  }

  double error_dist = error_distance(path_msg, vehicle_state);
  double conv_dist = convergence_distance(error_dist);
  unsigned int num_points = number_of_points(path_msg, conv_dist);
  
  unsigned int path_points_local = std::min((unsigned int)path_msg.pathpoint_array.size(), num_points);
  double factor = phoenician_factor(path_points_local);

  double x = vehicle_state.x;
  double y = vehicle_state.y;
  double v = vehicle_state.velocity_x;
  double yaw = vehicle_state.orientation;
  
  double alpha = 1.0;
  for (unsigned int i = 0; i < path_points_local; ++i) {
    custom_interfaces::msg::PathPoint& current_point = path_msg.pathpoint_array[i];
    double dx = current_point.x - x;
    double dy = current_point.y - y;
    double dv = current_point.v - v;
    double dyaw = current_point.orientation - yaw;

    current_point.x = x + (1 - alpha) * dx;
    current_point.y = y + (1 - alpha) * dy;
    current_point.v = v + (1 - alpha) * dv;
    current_point.orientation = yaw + (1 - alpha) * dyaw;

    alpha *= factor; // Adjust alpha based on the phoenician factor
  }
}