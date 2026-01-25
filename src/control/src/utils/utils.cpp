#include "utils/utils.hpp"

using namespace common_lib::structures;

Position rear_axis_position(
    const Position& cg, double orientation, double dist_cg_2_rear_axis) {
  Position rear_axis;
  rear_axis.x = cg.x - dist_cg_2_rear_axis * std::cos(orientation);
  rear_axis.y = cg.y - dist_cg_2_rear_axis * std::sin(orientation);
  return rear_axis;
}


std::tuple<Position, int, double> get_closest_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array, const Position& position) {
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
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
    int closest_point_id, double lookahead_distance, Position rear_axis_position, double last_to_first_max_dist) {

  if (pathpoint_array.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("control"), "Empty path");
    return std::make_tuple(Position(), 0.0, true);
  }

  // Clamp closest_point_id 
  closest_point_id = std::clamp(closest_point_id, 0, static_cast<int>(pathpoint_array.size()) - 1);

  Position rear_axis_point = rear_axis_position;

  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    size_t index_a = (static_cast<size_t>(closest_point_id) + i) % pathpoint_array.size();
    size_t index_b = (static_cast<size_t>(closest_point_id) + i + 1) % pathpoint_array.size();

    // We reached the end of the path
    if (index_b < index_a) {
      Position first_point(pathpoint_array.front().x, pathpoint_array.front().y);
      Position last_point(pathpoint_array.back().x, pathpoint_array.back().y);
      double start_end_distance = first_point.euclidean_distance(last_point);

      // If the path is not a closed track, force last point as lookahead
      // If the closest point is the first point, then the car is before the path start, so return first point, else return the last point
      if (start_end_distance > last_to_first_max_dist) {
        if (closest_point_id == 1) {
          // Car might be before the first point, return it as lookahead
          const auto &cp = pathpoint_array[static_cast<size_t>(closest_point_id)];
          return std::make_tuple(Position(cp.x, cp.y), cp.v, false);
        } else {
          RCLCPP_DEBUG(rclcpp::get_logger("control"),
                       "Lookahead extends beyond path end and it is not a closed track, using last "
                       "point of the path as lookahead point");
          return std::make_tuple(Position(pathpoint_array.back().x, pathpoint_array.back().y),
                                 pathpoint_array.back().v, false);
        }
      }
    }

    auto point_a = Position(pathpoint_array[index_a].x, pathpoint_array[index_a].y);
    auto point_b = Position(pathpoint_array[index_b].x, pathpoint_array[index_b].y);

    if (!(rear_axis_point.euclidean_distance(point_a) < lookahead_distance &&
          rear_axis_point.euclidean_distance(point_b) > lookahead_distance)) {
      continue;
    }

    double result_x;
    double result_y;

    // Slope of the line is infinite, don't need line equation
    if (point_a.x == point_b.x) {
      RCLCPP_DEBUG(rclcpp::get_logger("control"), "Vertical line!!");
      result_x = point_a.x;
      double delta = lookahead_distance * lookahead_distance - std::pow(result_x - rear_axis_point.x, 2);
      if (delta < 0) {
        continue;
      }
      double y1 = rear_axis_point.y + std::sqrt(delta);
      double y2 = rear_axis_point.y - std::sqrt(delta);

      if (y1 >= std::min(point_a.y, point_b.y) && y1 <= std::max(point_a.y, point_b.y)) {
        result_y = y1;
      } else {
        result_y = y2;
      }

    } else {
      // y = mx + c
      double m = (point_b.y - point_a.y) / (point_b.x - point_a.x);
      double c = point_a.y - m * point_a.x;

      // (x - x0)^2 + (y - y0)^2 = r^2
      // with x0 = rear_axis_point.x, y0 = rear_axis_point.y, r = lookahead_distance

      // with this information we can find the intersection point between the circle and the line
      // and obtain the lookahead point.
      // Expanded form of the circle equation with y substituted by mx + c gives a quadratic
      // equation:
      double A = 1 + m * m;
      double B = 2 * (m * c - rear_axis_point.x - m * rear_axis_point.y);
      double C = rear_axis_point.x * rear_axis_point.x + c * c +
                 rear_axis_point.y * rear_axis_point.y - 2 * c * rear_axis_point.y - lookahead_distance * lookahead_distance;

      double delta = B * B - 4 * A * C;

      if (delta < 0) {
        continue;
      }

      double x1 = (-B + std::sqrt(delta)) / (2 * A);
      double x2 = (-B - std::sqrt(delta)) / (2 * A);

      double y1 = m * x1 + c;
      double y2 = m * x2 + c;

      // solution (x,y) which is within bounds of point_a and point_b
      if (x1 >= std::min(point_a.x, point_b.x) && x1 <= std::max(point_a.x, point_b.x) &&
          y1 >= std::min(point_a.y, point_b.y) && y1 <= std::max(point_a.y, point_b.y)) {
        result_x = x1;
        result_y = y1;
      } else {
        result_x = x2;
        result_y = y2;
      }
    }

    return std::make_tuple(Position(result_x, result_y),
                           (pathpoint_array[index_a].v + pathpoint_array[index_b].v) / 2.0, false);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "No lookahead point found");
  return std::make_tuple(Position(), 0.0, true);
}
