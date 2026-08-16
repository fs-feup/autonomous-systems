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
  return 14.0 * std::log(error_distance + 1.0);
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

  auto& points = path_msg.pathpoint_array;

  // Arc length over which to rejoin the global path: scales with how far off the
  // car is, so small errors converge quickly and large ones get room to stay
  // feasible.
  const double error_dist = error_distance(path_msg, vehicle_state);
  const double conv_dist = convergence_distance(error_dist);
  const unsigned int num_points = number_of_points(path_msg, conv_dist);
  const unsigned int path_points_local = std::min((unsigned int)points.size(), num_points);

  // Car boundary state (hard requirement: the local path must start here exactly).
  const double x0 = vehicle_state.x;
  const double y0 = vehicle_state.y;
  const double yaw0 = vehicle_state.orientation;
  const double v0 = vehicle_state.velocity_x;

  // Not enough room to shape a spline: just pin the first point to the car.
  if (path_points_local < 3) {
    points[0].x = x0;
    points[0].y = y0;
    points[0].orientation = yaw0;
    points[0].v = v0;
    return;
  }

  // Rejoin point on the global path. Its position and heading are the far boundary
  // condition, so the local path leaves it tangent to the global path (G1) and the
  // remaining points (merge_idx+1 .. end) are already the global path, untouched.
  const unsigned int merge_idx = path_points_local - 1;
  const double x1 = points[merge_idx].x;
  const double y1 = points[merge_idx].y;
  const double yaw1 = points[merge_idx].orientation;

  // Cubic Bezier in Hermite form: it matches position AND heading at both ends, so
  // the path leaves the car along the car's orientation and arrives tangent to the
  // global path -- exactly the gradual, feasible convergence we want (no start hook,
  // no opposite-side overshoot). Handle length ~1/3 of the chord keeps curvature low
  // and near-circular; a nearly-aligned car/path yields a nearly-straight blend.
  const double chord = std::hypot(x1 - x0, y1 - y0);
  const double handle = chord / 3.0;
  const double b0x = x0;
  const double b0y = y0;
  const double b1x = x0 + handle * std::cos(yaw0);
  const double b1y = y0 + handle * std::sin(yaw0);
  const double b2x = x1 - handle * std::cos(yaw1);
  const double b2y = y1 - handle * std::sin(yaw1);
  const double b3x = x1;
  const double b3y = y1;

  for (unsigned int i = 0; i <= merge_idx; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(merge_idx);
    const double u = 1.0 - t;

    const double bx = u * u * u * b0x + 3 * u * u * t * b1x + 3 * u * t * t * b2x + t * t * t * b3x;
    const double by = u * u * u * b0y + 3 * u * u * t * b1y + 3 * u * t * t * b2y + t * t * t * b3y;

    // Bezier derivative -> tangent heading (starts at yaw0, ends at yaw1 by construction).
    const double dx = 3 * u * u * (b1x - b0x) + 6 * u * t * (b2x - b1x) + 3 * t * t * (b3x - b2x);
    const double dy = 3 * u * u * (b1y - b0y) + 6 * u * t * (b2y - b1y) + 3 * t * t * (b3y - b2y);

    custom_interfaces::msg::PathPoint& p = points[i];
    p.x = bx;
    p.y = by;
    if (dx * dx + dy * dy > 1e-12) {
      p.orientation = std::atan2(dy, dx);
    }

    // Only the first point takes the car's velocity (it anchors the reference ramp
    // and is a hard requirement). The rest keep the planner's feasibility-shaped
    // profile; blending them toward the car's speed erases the speed error signal
    // and the controller stagnates below the planned velocity in corners.
    if (i == 0) {
      p.v = v0;
    }
  }
}