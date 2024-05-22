#include "point_solver/psolver.hpp"

/**
 * @brief PointSolver Constructer
 */
PointSolver::PointSolver() = default;

/**
 * @brief Update vehicle pose
 *
 * @param pose msg
 */
void PointSolver::update_vehicle_pose(
    const custom_interfaces::msg::VehicleState::ConstSharedPtr &vehicle_state_msg) {
  // update to Rear Wheel position
  this->vehicle_pose_.cg_.x_ = vehicle_state_msg->position.x;
  this->vehicle_pose_.cg_.y_ = vehicle_state_msg->position.y;

  this->vehicle_pose_.velocity_ = vehicle_state_msg->linear_velocity;
  this->vehicle_pose_.heading_ = vehicle_state_msg->theta;
  this->vehicle_pose_.rear_axis_ = cg_2_rear_axis(
      this->vehicle_pose_.cg_, this->vehicle_pose_.heading_, this->dist_cg_2_rear_axis_);

  return;
}

/**
 * @brief Find the closest point on the path
 *
 * @param path
 */
std::pair<Point, int> PointSolver::update_closest_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array,
    Point rear_axis_point) const {
  double min_distance = 1e9;
  Point closest_point = Point();
  Point aux_point = Point();
  int closest_point_id = -1;
  for (size_t i = 0; i < pathpoint_array.size(); i++) {
    aux_point = Point(pathpoint_array[i].x, pathpoint_array[i].y);
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
 * @return std::pair<Point, int> lookahead point and error status (1 = error)
 */
std::tuple<Point, double, bool> PointSolver::update_lookahead_point(
    const std::vector<custom_interfaces::msg::PathPoint> &pathpoint_array, Point rear_axis_point,
    int closest_point_id, double ld, double ld_margin) const {
  Point lookahead_point = Point();
  Point aux_point = Point();
  size_t size = pathpoint_array.size();
  for (size_t i = 0; i < size; i++) {
    size_t index = (closest_point_id + i) % size;
    aux_point = Point(pathpoint_array[index].x, pathpoint_array[index].y);
    double distance = rear_axis_point.euclidean_distance(aux_point);
    if (std::abs(distance - ld) <= (ld * ld_margin)) {
      lookahead_point = aux_point;
      return std::make_tuple(lookahead_point, pathpoint_array[index].v, 0);
    }
  }
  return std::make_tuple(lookahead_point, 0, 1);
}

/**
 * @brief update the LookaheadDistance based on a new velocity
 */
double PointSolver::update_lookahead_distance(double k, double velocity) const {
  return k * velocity;
};

Point PointSolver::cg_2_rear_axis(Point cg, double heading, double dist_cg_2_rear_axis) const {
  Point rear_axis = Point();
  rear_axis.x_ = cg.x_ - dist_cg_2_rear_axis * cos(heading);
  rear_axis.y_ = cg.y_ - dist_cg_2_rear_axis * sin(heading);
  return rear_axis;
}