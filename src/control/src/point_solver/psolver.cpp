#include "point_solver/psolver.hpp"

using namespace common_lib::structures;
using namespace common_lib::vehicle_dynamics;

/**
 * @brief PointSolver Constructer
 */
PointSolver::PointSolver(double k, double ld_margin) : k_(k), ld_margin_(ld_margin) {}

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
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
    Position rear_axis_point) const {
  double min_distance = 1e9;
  Position closest_point = Position();
  Position aux_point = Position();
  int closest_point_id = -1;
  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    aux_point = Position(pathpoint_array[i].x, pathpoint_array[i].y);
    double distance = rear_axis_point.euclidean_distance(aux_point);
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
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array, Position rear_axis_point,
    int closest_point_id) const {
  Position lookahead_point = Position();

  double closest_distance = std::numeric_limits<double>::max();
  custom_interfaces::msg::PathPoint closest_yet;

  double ld = this->k_ * std::max(this->vehicle_pose_.velocity_, 2.0);
  RCLCPP_DEBUG(rclcpp::get_logger("control"), "Current ld: %f", ld);

  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    size_t index = (closest_point_id + i) % pathpoint_array.size();
    lookahead_point = Position(pathpoint_array[index].x, pathpoint_array[index].y);
    double distance = rear_axis_point.euclidean_distance(lookahead_point);

    // if the distance is within the margin, return the point
    if (std::abs(distance - ld) <= (ld * this->ld_margin_)) {
      return std::make_tuple(lookahead_point, pathpoint_array[index].v, false);
    }

    // if the distance is less than the lookahead distance (we don't want a lookahead further than
    // ld if it's not withing ld margin), check if it is the closest point found yet
    if (distance < ld && std::abs(ld - distance) < closest_distance) {
      closest_distance = std::abs(ld - distance);
      closest_yet = pathpoint_array[index];
    }
  }

  if (closest_distance == std::numeric_limits<double>::max()) {
    RCLCPP_WARN(rclcpp::get_logger("control"), "No lookahead point found");
    return std::make_tuple(Position(), 0.0, true);
  }

  double scaled_velocity =
      closest_yet.v * rear_axis_point.euclidean_distance(Position(closest_yet.x, closest_yet.y)) /
      ld;
  return std::make_tuple(Position(closest_yet.x, closest_yet.y), scaled_velocity, false);
}

/**
 * @brief update the LookaheadDistance based on a new velocity
 */
double PointSolver::update_lookahead_distance(double k, double velocity) const {
  return k * velocity;
};

