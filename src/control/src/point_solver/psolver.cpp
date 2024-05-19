#include "point_solver/psolver.hpp"

/**
 * @brief PointSolver Constructer
 */
PointSolver::PointSolver() {}

/**
 * @brief Update vehicle pose
 *
 * @param pose msg
 */
void PointSolver::update_vehicle_pose(
    const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg) {
  // update to Rear Wheel position
  this->vehicle_pose_.cg_.x_ = pose_msg->position.x;
  this->vehicle_pose_.cg_.y_ = pose_msg->position.y;

  this->vehicle_pose_.velocity_ = pose_msg->velocity;
  this->vehicle_pose_.heading_ = pose_msg->theta;
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
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg, Point rear_axis_point) {
  double min_distance = 1e9;
  Point closest_point = Point();
  Point aux_point = Point();
  int closest_point_id = -1;
  for (long unsigned int i = 0; i < path_msg->pathpoint_array.size(); i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = rear_axis_point.euclidean_distance(aux_point);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = aux_point;
      closest_point_id = i;
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
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg, Point rear_axis_point,
    int closest_point_id, double ld, double ld_margin) {
  Point lookahead_point = Point();
  Point aux_point = Point();
  for (int i = closest_point_id; i < path_msg->pathpoint_array.size(); i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = rear_axis_point.euclidean_distance(aux_point);
    if (std::abs(distance - ld) <= (ld * ld_margin)) {
      lookahead_point = aux_point;
      return std::make_tuple(lookahead_point, path_msg->pathpoint_array[i].v, 0);
    }
  }
  for (int i = 0; i < closest_point_id; i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = rear_axis_point.euclidean_distance(aux_point);
    if (std::abs(distance - ld) <= (ld * ld_margin)) {
      lookahead_point = aux_point;
      return std::make_tuple(lookahead_point, path_msg->pathpoint_array[i].v, 0);
    }
  }
  return std::make_tuple(lookahead_point, 0, 1);
}

/**
 * @brief update the LookaheadDistance based on a new velocity
 */
double PointSolver::update_lookahead_distance(double k, double velocity) { return k * velocity; }

Point PointSolver::cg_2_rear_axis(Point cg, double heading, double dist_cg_2_rear_axis) {
  Point rear_axis = Point();
  rear_axis.x_ = cg.x_ - dist_cg_2_rear_axis * cos(heading);
  rear_axis.y_ = cg.y_ - dist_cg_2_rear_axis * sin(heading);
  return rear_axis;
}