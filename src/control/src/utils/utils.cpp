#include "utils/utils.hpp"

using namespace common_lib::structures;

Position rear_axis_position(const Position& cg, double orientation, double dist_cg_2_rear_axis) {
  Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * std::cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * std::sin(orientation);
  return rear_axis;
}

std::tuple<Position, int, double> get_closest_point(
    const std::vector<custom_interfaces::msg::PathPoint>& pathpoint_array,
    const Position& position) {
  double min_distance = 1e9;
  double closest_point_velocity = 0;
  Position closest_point = Position();
  Position aux_point = Position();
  int closest_point_id = -1;
  for (size_t i = 1; i < pathpoint_array.size(); i++) {
    aux_point = Position(pathpoint_array[i].x, pathpoint_array[i].y);
    double distance = position.euclidean_distance(aux_point);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = aux_point;
      closest_point_id = static_cast<int>(i);
      closest_point_velocity = pathpoint_array[i].v;
    }
  }
  return std::make_tuple(closest_point, closest_point_id, closest_point_velocity);
}

std::tuple<Position, double, bool> get_lookahead_point(
    const std::vector<custom_interfaces::msg::PathPoint>& pathpoint_array, int closest_point_id,
    double lookahead_distance, Position rear_axis_position, double last_to_first_max_dist) {
  using PathPoint = custom_interfaces::msg::PathPoint;

  const size_t N = pathpoint_array.size();
  if (N == 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("control"), "Empty path");
    return std::make_tuple(Position(), 0.0, true);
  }
  if (N == 1) {
    return std::make_tuple(Position(pathpoint_array.front().x, pathpoint_array.front().y),
                           pathpoint_array.front().v, false);
  }

  auto dist = [](double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1, dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  };
  auto clamp01 = [](double t) { return std::max(0.0, std::min(1.0, t)); };

  const Position first_point(pathpoint_array.front().x, pathpoint_array.front().y);
  const Position last_point(pathpoint_array.back().x, pathpoint_array.back().y);
  const bool is_closed = (first_point.euclidean_distance(last_point) <= last_to_first_max_dist);

  // --- Build cumulative arc-length s[i] (0 at point 0)
  std::vector<double> s(N, 0.0);
  for (size_t i = 1; i < N; ++i) {
    s[i] = s[i - 1] + dist(pathpoint_array[i - 1].x, pathpoint_array[i - 1].y, pathpoint_array[i].x,
                           pathpoint_array[i].y);
  }
  const double total_len_open = s.back();
  const double closing_seg_len = is_closed
                                     ? dist(pathpoint_array.back().x, pathpoint_array.back().y,
                                            pathpoint_array.front().x, pathpoint_array.front().y)
                                     : 0.0;
  const double total_len = is_closed ? (total_len_open + closing_seg_len) : total_len_open;

  // --- Guard against degenerate path with zero length everywhere
  if (total_len <= 1e-9) {
    RCLCPP_DEBUG(rclcpp::get_logger("control"), "Path total length ~ 0");
    return std::make_tuple(Position(pathpoint_array.front().x, pathpoint_array.front().y),
                           pathpoint_array.front().v, false);
  }

  // --- Compute starting abscissa s0 by projecting rear axle onto segment [closest, next]
  const size_t a = static_cast<size_t>(std::clamp(closest_point_id, 0, static_cast<int>(N - 1)));
  const size_t b =
      (a + 1 < N) ? (a + 1) : (is_closed ? 0 : a);  // for open path, last segment doesn't exist

  const double ax = pathpoint_array[a].x, ay = pathpoint_array[a].y;
  const double bx = pathpoint_array[b].x, by = pathpoint_array[b].y;
  const double abx = bx - ax, aby = by - ay;
  const double ab2 = abx * abx + aby * aby;

  double t = 0.0;
  if (ab2 > 0.0 && b != a) {
    const double apx = rear_axis_position.x - ax;
    const double apy = rear_axis_position.y - ay;
    t = clamp01((apx * abx + apy * aby) / ab2);
  }

  double seg_len_ab = (b == a) ? 0.0 : std::sqrt(ab2);
  double s0 = s[a] + t * seg_len_ab;

  // --- Target abscissa s* forward by lookahead_distance
  double s_star = s0 + lookahead_distance;

  if (!is_closed) {
    if (s_star >= total_len) {
      // Beyond end of open path: clamp to last point
      RCLCPP_DEBUG(rclcpp::get_logger("control"),
                   "Lookahead beyond open path end; using last point");
      return std::make_tuple(Position(pathpoint_array.back().x, pathpoint_array.back().y),
                             pathpoint_array.back().v, false);
    }
    // also guard lower bound (shouldn't happen with positive lookahead)
    if (s_star <= 0.0) s_star = 0.0;
  } else {
    // Wrap around closed track
    s_star = std::fmod(s_star, total_len);
    if (s_star < 0.0) s_star += total_len;
  }

  // --- Find segment containing s_star and interpolate (x,y,v)
  auto interp_on_segment = [&](size_t i0, size_t i1, double s_left,
                               double seg_len) -> std::tuple<Position, double> {
    const double x0 = pathpoint_array[i0].x, y0 = pathpoint_array[i0].y, v0 = pathpoint_array[i0].v;
    const double x1 = pathpoint_array[i1].x, y1 = pathpoint_array[i1].y, v1 = pathpoint_array[i1].v;
    double alpha = 0.0;
    if (seg_len > 1e-12) alpha = (s_star - s_left) / seg_len;
    alpha = std::max(0.0, std::min(1.0, alpha));
    const double xi = x0 + (x1 - x0) * alpha;
    const double yi = y0 + (y1 - y0) * alpha;
    const double vi = v0 + (v1 - v0) * alpha;
    return {Position(xi, yi), vi};
  };

  // Handle interior segments [i, i+1] for i in [0..N-2]
  auto it = std::upper_bound(s.begin(), s.end(), s_star);
  if (it != s.begin() && it != s.end()) {
    size_t i = static_cast<size_t>(std::distance(s.begin(), it) - 1);
    size_t j = i + 1;
    const double s_left = s[i];
    const double seg_len = s[j] - s[i];
    auto [pos, v] = interp_on_segment(i, j, s_left, seg_len);
    return std::make_tuple(pos, v, false);
  }

  // Edge cases:
  if (it == s.begin()) {
    // s_star at/near the very start
    size_t i = 0, j = 1;
    const double seg_len = s[j] - s[i];
    auto [pos, v] = interp_on_segment(i, j, 0.0, seg_len);
    return std::make_tuple(pos, v, false);
  }

  // it == s.end(): s_star at/after last recorded cumulative point
  if (!is_closed) {
    // Open path: clamp to last
    return std::make_tuple(Position(pathpoint_array.back().x, pathpoint_array.back().y),
                           pathpoint_array.back().v, false);
  } else {
    // Closed path: could be on the closing segment [N-1 -> 0]
    const double s_left = s.back();
    const double seg_len = closing_seg_len;
    // If closing segment is degenerate, just return first point
    if (seg_len <= 1e-12) {
      return std::make_tuple(Position(pathpoint_array.front().x, pathpoint_array.front().y),
                             pathpoint_array.front().v, false);
    }
    auto [pos, v] = interp_on_segment(N - 1, 0, s_left, seg_len);
    return std::make_tuple(pos, v, false);
  }

  // Fallback (shouldn't hit)
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "No lookahead point found (arc-length)");
  return std::make_tuple(Position(), 0.0, true);
}
