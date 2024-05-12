#include "pure_pursuit/pp.hpp"

/**
 * @brief Pure Pursuit class Constructor
 *
 * @param k Lookahead gain
 */

PurePursuit::PurePursuit(double k, double ld_margin)
    : k_(k),
      ld_margin_(ld_margin),
      wheel_base_(WHEEL_BASE),
      dist_cg_2_rear_axis_(DIST_CG_2_REAR_AXIS),
      vehicle_pose_(Pose()),
      lookahead_point_(Point()),
      closest_point_(Point()),
      closest_point_id_(-1) {}

double PurePursuit::update_steering_angle(
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg,
    const custom_interfaces::msg::Pose::ConstSharedPtr &pose_msg) {
  update_vehicle_pose(pose_msg);

  this->ld_ = this->update_lookahead_distance();

  // update closest point
  auto closest_point_info = this->update_closest_point(path_msg);
  this->closest_point_ = closest_point_info.first;
  this->closest_point_id_ = closest_point_info.second;

  // update lookahead point
  auto lookahead_point_info = this->update_lookahead_point(path_msg);
  bool error = lookahead_point_info.second;
  if (error) {
    // RCLCPP_ERROR(this->get_logger(), "PurePursuit: Failed to update lookahed point");
    /*
     * isto pode acontecer por termos o parametro ld_margin demasiado pequeno com base
     * no numero de pontos que existe no path
     */
  }
  this->lookahead_point_ = lookahead_point_info.first;

  // calculate steering angle
  return this->pp_steering_control_law();
}

/**
 * @brief update the LookaheadDistance based on a new velocity
 */
double PurePursuit::update_lookahead_distance() { return this->k_ * this->vehicle_pose_.velocity_; }

/**
 * @brief Update vehicle pose
 *
 * @param pose msg
 */
void PurePursuit::update_vehicle_pose(
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
std::pair<Point, int> PurePursuit::update_closest_point(
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg) {
  double min_distance = 1e9;
  Point closest_point = Point();
  Point aux_point = Point();
  int closest_point_id = -1;
  for (long unsigned int i = 0; i < path_msg->pathpoint_array.size(); i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = this->vehicle_pose_.rear_axis_.euclidean_distance(aux_point);
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
std::pair<Point, bool> PurePursuit::update_lookahead_point(
    const custom_interfaces::msg::PathPointArray::ConstSharedPtr &path_msg) {
  Point lookahead_point = Point();
  Point aux_point = Point();
  for (int i = this->closest_point_id_; i < path_msg->pathpoint_array.size(); i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = this->closest_point_.euclidean_distance(aux_point);
    if (distance <= this->ld_ * (1 - this->ld_margin_) &&
        distance >= this->ld_ * (1 + this->ld_margin_)) {
      lookahead_point = aux_point;
      return std::make_pair(lookahead_point, 0);
    }
  }
  for (int i = 0; i < this->closest_point_id_; i++) {
    aux_point = Point(path_msg->pathpoint_array[i].x, path_msg->pathpoint_array[i].y);
    double distance = this->closest_point_.euclidean_distance(aux_point);
    if (distance > this->ld_) {
      lookahead_point = aux_point;
      return std::make_pair(lookahead_point, 0);
    }
  }
  return std::make_pair(lookahead_point, 1);
}

double PurePursuit::pp_steering_control_law() {
  double alpha = calculate_alpha(this->vehicle_pose_.rear_axis_, this->vehicle_pose_.cg_,
                                 this->lookahead_point_, this->dist_cg_2_rear_axis_);

  // update lookahead distance to the actual distance
  this->ld_ = this->vehicle_pose_.rear_axis_.euclidean_distance(this->lookahead_point_);

  return atan(2 * this->wheel_base_ * sin(alpha) / this->ld_);
}

double PurePursuit::calculate_alpha(Point vehicle_rear_wheel, Point vehicle_cg,
                                    Point lookahead_point, double dist_cg_2_rear_axis) {
  double lookhead_point_2_rear_wheel = vehicle_rear_wheel.euclidean_distance(lookahead_point);
  double lookhead_point_2_cg = lookahead_point.euclidean_distance(vehicle_cg);

  // Law of cosines
  double alpha = acos((pow(lookhead_point_2_rear_wheel, 2) + pow(dist_cg_2_rear_axis, 2) -
                       pow(lookhead_point_2_cg, 2)) /
                      (2 * lookhead_point_2_rear_wheel * dist_cg_2_rear_axis));

  return alpha;
}

Point PurePursuit::cg_2_rear_axis(Point cg, double heading, double dist_cg_2_rear_axis) {
  Point rear_axis = Point();
  rear_axis.x_ = cg.x_ - dist_cg_2_rear_axis * cos(heading);
  rear_axis.y_ = cg.y_ - dist_cg_2_rear_axis * sin(heading);
  return rear_axis;
}