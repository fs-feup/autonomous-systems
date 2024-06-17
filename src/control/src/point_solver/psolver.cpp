#include "point_solver/psolver.hpp"

using namespace common_lib::structures;
using namespace common_lib::vehicle_dynamics;

/**
 * @brief PointSolver Constructer
 */
PointSolver::PointSolver(double k) : k_(k) {}

/**
 * @brief Update vehicle pose
 *
 * @param pose msg
 */
void PointSolver::update_vehicle_pose(
    const custom_interfaces::msg::VehicleState &vehicle_state_msg) {
  // update to Rear Wheel position
  this->vehicle_pose_.position.x = vehicle_state_msg.position.x;
  this->vehicle_pose_.position.y = vehicle_state_msg.position.y;

  this->vehicle_pose_.velocity_ = vehicle_state_msg.linear_velocity;
  this->vehicle_pose_.orientation = vehicle_state_msg.theta;
  RCLCPP_INFO(rclcpp::get_logger("control"),
              "Calculating rear axis: CG.x %f CG.y %f, orientation %f, Dist cg 2 rear axis %f",
              this->vehicle_pose_.position.x, vehicle_pose_.position.y,
              this->vehicle_pose_.orientation, this->dist_cg_2_rear_axis_);
  this->vehicle_pose_.rear_axis_ = cg_2_rear_axis(
      this->vehicle_pose_.position, this->vehicle_pose_.orientation, this->dist_cg_2_rear_axis_);

  RCLCPP_INFO(rclcpp::get_logger("control"), "Current rear axis: %f, %f",
              vehicle_pose_.rear_axis_.x, vehicle_pose_.rear_axis_.y);
  return;
}

/**
 * @brief Find the closest point on the path
 *
 * @param path
 */
std::pair<Position, int> PointSolver::update_closest_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array) const {
  double min_distance = 1e9;
  Position closest_point = Position();
  Position aux_point = Position();
  int closest_point_id = -1;
  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    aux_point = Position(pathpoint_array[i].x, pathpoint_array[i].y);
    double distance = this->vehicle_pose_.rear_axis_.euclidean_distance(aux_point);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = aux_point;
      closest_point_id = static_cast<int>(i);
    }
  }
  return std::make_pair(closest_point, closest_point_id);
}

/**
 * @brief Update Lookahead point
 *
 * @param path
 * @return std::pair<Position, int> lookahead point and error status (1 = error)
 */
std::tuple<Position, double, bool> PointSolver::update_lookahead_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
    int closest_point_id) const {
  Position rear_axis_point = this->vehicle_pose_.rear_axis_;
  double ld = std::max(this->k_ * this->vehicle_pose_.velocity_, 2.5);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Current ld: %f", ld);

  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    size_t index_a = (closest_point_id + i) % pathpoint_array.size();
    size_t index_b = (closest_point_id + i + 1) % pathpoint_array.size();
    auto point_a = Position(pathpoint_array[index_a].x, pathpoint_array[index_a].y);
    auto point_b = Position(pathpoint_array[index_b].x, pathpoint_array[index_b].y);

    if (!(rear_axis_point.euclidean_distance(point_a) < ld &&
          rear_axis_point.euclidean_distance(point_b) > ld)) {
      continue;
    }

    if (point_a.x == point_b.x) {
      continue;
    }

    // y = mx + c
    double m = (point_b.y - point_a.y) / (point_b.x - point_a.x);
    double c = point_a.y - m * point_a.x;

    // (x - x0)^2 + (y - y0)^2 = r^2
    // with x0 = rear_axis_point.x, y0 = rear_axis_point.y, r = ld

    // with this information we can find the intersection point between the circle and the line and
    // obtain the lookahead point Expanded form of the circle equation with y substituted by mx + c
    // gives a quadratic equation:
    double A = 1 + m * m;
    double B = 2 * (m * c - rear_axis_point.x - m * rear_axis_point.y);
    double C = rear_axis_point.x * rear_axis_point.x + c * c +
               rear_axis_point.y * rear_axis_point.y - 2 * c * rear_axis_point.y - ld * ld;

    double delta = B * B - 4 * A * C;

    if (delta < 0) {
      continue;
    }

    double x1 = (-B + std::sqrt(delta)) / (2 * A);
    double x2 = (-B - std::sqrt(delta)) / (2 * A);

    double y1 = m * x1 + c;
    double y2 = m * x2 + c;

    double x;
    double y;

    // i want to select the solution (x,y) which is within bounds of point_a and point_b
    if (x1 >= std::min(point_a.x, point_b.x) && x1 <= std::max(point_a.x, point_b.x) &&
        y1 >= std::min(point_a.y, point_b.y) && y1 <= std::max(point_a.y, point_b.y)) {
      x = x1;
      y = y1;
    } else {
      x = x2;
      y = y2;
    }

    RCLCPP_INFO(rclcpp::get_logger("control"), "Point A: x: %f, y: %f", point_a.x, point_a.y);
    RCLCPP_INFO(rclcpp::get_logger("control"), "Point B: x: %f, y: %f", point_b.x, point_b.y);
    RCLCPP_INFO(rclcpp::get_logger("control"), "Lookahead point: x: %f, y: %f", x, y);

    return std::make_tuple(Position(x, y),
                           (pathpoint_array[index_a].v + pathpoint_array[index_a].v) / 2.0, false);

    // if the distance is less than the lookahead distance (we don't want a lookahead further than
    // ld if it's not withing ld margin), check if it is the closest point found yet
    // if (distance > 1 && std::abs(ld - distance) < closest_distance) {
    //   RCLCPP_INFO(rclcpp::get_logger("control"), "New closest point found that doesnt fint
    //   margin: %f", distance); closest_distance = std::abs(ld - distance); closest_yet =
    //   pathpoint_array[index];
    // }
  }
  return std::make_tuple(Position(), 0.0, true);
  // RCLCPP_INFO(rclcpp::get_logger("control"), "Rear axis point: x: %f, y: %f", rear_axis_point.x,
  // rear_axis_point.y);

  // if (closest_distance == std::numeric_limits<double>::max()) {
  //   RCLCPP_WARN(rclcpp::get_logger("control"), "No lookahead point found");
  //   return std::make_tuple(Position(), 0.0, true);
  // }

  // // RCLCPP_INFO(rclcpp::get_logger("control"), "Closest yet point: x: %f, y: %f, v: %f",
  // closest_yet.x, closest_yet.y, closest_yet.v); double scaled_velocity =
  //     closest_yet.v * rear_axis_point.euclidean_distance(Position(closest_yet.x, closest_yet.y))
  //     / ld;
  // return std::make_tuple(Position(closest_yet.x, closest_yet.y), scaled_velocity, false);
}

/**
 * @brief update the LookaheadDistance based on a new velocity
 */
double PointSolver::update_lookahead_distance(double k, double velocity) const {
  return k * velocity;
};
