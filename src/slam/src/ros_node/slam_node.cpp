#include "ros_node/slam_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "common_lib/communication/marker.hpp"
#include "common_lib/maths/transformations.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "motion_lib/v2p_models/map.hpp"
#include "perception_sensor_lib/data_association/map.hpp"
#include "perception_sensor_lib/landmark_filter/map.hpp"
#include "perception_sensor_lib/loop_closure/lap_counter.hpp"
#include "slam_solver/map.hpp"

/*---------------------- Constructor --------------------*/

SLAMNode::SLAMNode(const SLAMParameters &params) : Node("slam") {
  // Print parameters
  // Initialize the models
  std::shared_ptr<V2PMotionModel> motion_model = v2p_models_map.at(params.motion_model_name_)();
  std::shared_ptr<DataAssociationModel> data_association =
      data_association_models_map.at(params.data_association_model_name_)(DataAssociationParameters(
          params.data_association_limit_distance_, params.data_association_gate_,
          params.new_landmark_confidence_gate_, params.observation_x_noise_,
          params.observation_y_noise_));
  std::shared_ptr<LandmarkFilter> landmark_filter =
      landmark_filters_map.at(params.landmark_filter_name_)(
          LandmarkFilterParameters(params.minimum_observation_count_,
                                   params.minimum_frequency_of_detections_),
          data_association);

  this->_execution_times_ = std::make_shared<std::vector<double>>(20, 0.0);
  std::shared_ptr<LoopClosure> loop_closure = std::make_shared<LapCounter>(4, 10, 5, 3);

  // Initialize SLAM solver object
  this->_slam_solver_ = slam_solver_constructors_map.at(params.slam_solver_name_)(
      params, data_association, motion_model, landmark_filter, this->_execution_times_,
      loop_closure);

  _perception_map_ = std::vector<common_lib::structures::Cone>();
  _vehicle_state_velocities_ = common_lib::structures::Velocities();
  _track_map_ = std::vector<common_lib::structures::Cone>();
  _vehicle_pose_ = common_lib::structures::Pose();

  // Subscriptions
  if (!params.use_simulated_perception_) {
    this->_perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 1,
        std::bind(&SLAMNode::_perception_subscription_callback, this, std::placeholders::_1));
  }
  if (!params.use_simulated_velocities_) {
    this->_velocities_subscription_ = this->create_subscription<custom_interfaces::msg::Velocities>(
        "/state_estimation/velocities", 1,
        std::bind(&SLAMNode::_velocities_subscription_callback, this, std::placeholders::_1));
  }

  // Publishers
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
  this->_vehicle_pose_publisher_ =
      this->create_publisher<custom_interfaces::msg::Pose>("/state_estimation/vehicle_pose", 10);
  this->_visualization_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map", 10);
  this->_visualization_perception_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map_perception", 10);
  this->_associations_visualization_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_associations", 10);
  this->_position_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/state_estimation/visualization/position", 10);
  this->_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/state_estimation/slam_execution_time", 10);
  this->_covariance_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/state_estimation/slam_covariance", 10);
  this->_lap_counter_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/state_estimation/lap_counter", 10);

  this->_perception_delta_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/perception_delta", 10);

  this->_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void SLAMNode::init() { this->_slam_solver_->init(this->weak_from_this()); }

/*---------------------- Subscriptions --------------------*/

void SLAMNode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;

  if (!this->_go_ || this->_mission_ == common_lib::competition_logic::Mission::NONE) {
    return;
  }

  rclcpp::Time start_time = this->get_clock()->now();
  rclcpp::Time current_stamp(msg.header.stamp);
  if (_last_perception_message_time_.nanoseconds() != 0) {
    double delta_ms = (current_stamp - _last_perception_message_time_).seconds() * 1000.0;

    std_msgs::msg::Float64 msg;
    msg.data = delta_ms;
    _perception_delta_publisher_->publish(msg);
  }

  // Update last timestamp
  _last_perception_message_time_ = current_stamp;

  this->_perception_map_.clear();
  for (auto &cone : cone_array) {
    this->_perception_map_.push_back(common_lib::structures::Cone(
        cone.position.x, cone.position.y, cone.color, cone.confidence, msg.header.stamp));
  }

  if (this->_slam_solver_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ATTR - Slam Solver object is null");
    return;
  }

  this->_slam_solver_->add_observations(this->_perception_map_);
  this->_track_map_ = this->_slam_solver_->get_map_estimate();
  this->_vehicle_pose_ = this->_slam_solver_->get_pose_estimate();
  this->_associations_ = this->_slam_solver_->get_associations();
  this->_observations_global_ = this->_slam_solver_->get_observations_global();
  this->_map_coordinates_ = this->_slam_solver_->get_map_coordinates();

  if (this->_is_mission_finished()) {
    this->finish();
  }

  // this->_publish_covariance(); // TODO: get covariance to work fast

  this->_publish_vehicle_pose();
  this->_publish_map();
  this->_publish_lap_counter();
  this->_publish_associations();

  // Timekeeping
  rclcpp::Time end_time = this->get_clock()->now();
  this->_execution_times_->at(1) = (end_time - start_time).seconds() * 1000.0;

  std_msgs::msg::Float64MultiArray execution_time_msg;
  execution_time_msg.data = *this->_execution_times_;
  this->_execution_time_publisher_->publish(execution_time_msg);
}

void SLAMNode::_velocities_subscription_callback(const custom_interfaces::msg::Velocities &msg) {
  if (this->_mission_ == common_lib::competition_logic::Mission::NONE) {
    return;
  }
  rclcpp::Time start_time = this->get_clock()->now();
  rclcpp::Time time1, time2, time3, time4, time5, time6;

  this->_vehicle_state_velocities_ = common_lib::structures::Velocities(
      msg.velocity_x, msg.velocity_y, msg.angular_velocity, msg.covariance[0], msg.covariance[4],
      msg.covariance[8], msg.header.stamp);
  time2 = this->get_clock()->now();

  this->_slam_solver_->add_motion_prior(this->_vehicle_state_velocities_);
  time3 = this->get_clock()->now();
  this->_vehicle_pose_ = this->_slam_solver_->get_pose_estimate();
  time4 = this->get_clock()->now();
  this->_track_map_ = this->_slam_solver_->get_map_estimate();
  time5 = this->get_clock()->now();

  // this->_publish_covariance(); // TODO: get covariance to work fast
  this->_publish_vehicle_pose();
  time6 = this->get_clock()->now();

  rclcpp::Time end_time = this->get_clock()->now();
  this->_execution_times_->at(0) = (end_time - start_time).seconds() * 1000.0;
  this->_execution_times_->at(11) = (time2 - start_time).seconds() * 1000.0;
  this->_execution_times_->at(12) = (time3 - time2).seconds() * 1000.0;
  this->_execution_times_->at(13) = (time4 - time3).seconds() * 1000.0;
  this->_execution_times_->at(14) = (time5 - time4).seconds() * 1000.0;
  this->_execution_times_->at(15) = (time6 - time5).seconds() * 1000.0;
  std_msgs::msg::Float64MultiArray execution_time_msg;
  execution_time_msg.data = *this->_execution_times_;
  this->_execution_time_publisher_->publish(execution_time_msg);
}

/*---------------------- Publications --------------------*/

void SLAMNode::_publish_vehicle_pose() {
  auto message = custom_interfaces::msg::Pose();
  auto tf_message = geometry_msgs::msg::TransformStamped();
  message.x = this->_vehicle_pose_.position.x;
  message.y = this->_vehicle_pose_.position.y;
  message.theta = this->_vehicle_pose_.orientation;
  // TODO(marhcouto): add covariance
  message.header.stamp = this->get_clock()->now();

  this->_vehicle_pose_publisher_->publish(message);

  // Publish the transform
  tf_message.header.stamp = message.header.stamp;
  tf_message.header.frame_id = "map";
  tf_message.child_frame_id = "vehicle_estimate";

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, message.theta);
  tf_message.transform.translation.x = message.x;
  tf_message.transform.translation.y = message.y;
  tf_message.transform.translation.z = 0.0;
  tf_message.transform.rotation.x = q.x();
  tf_message.transform.rotation.y = q.y();
  tf_message.transform.rotation.z = q.z();
  tf_message.transform.rotation.w = q.w();
  this->_tf_broadcaster_->sendTransform(tf_message);
}

void SLAMNode::_publish_map() {
  auto cone_array_msg = custom_interfaces::msg::ConeArray();
  auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  auto marker_array_msg_perception = visualization_msgs::msg::MarkerArray();
  for (common_lib::structures::Cone const &cone : this->_track_map_) {
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position.x = cone.position.x;
    cone_message.position.y = cone.position.y;
    // TODO(marhcouto): add covariance & large cones & confidence
    cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
    cone_array_msg.cone_array.push_back(cone_message);
    // RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
    //              cone_message.position.y, cone_message.color.c_str());
  }
  // RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  cone_array_msg.header.stamp = this->get_clock()->now();
  this->_map_publisher_->publish(cone_array_msg);
  marker_array_msg = common_lib::communication::marker_array_from_structure_array(
      this->_track_map_, "map_cones", "map");
  this->_visualization_map_publisher_->publish(marker_array_msg);
}

void SLAMNode::_publish_covariance() {
  // Publish the covariance
  std_msgs::msg::Float64MultiArray covariance_msg;
  covariance_msg.layout = std_msgs::msg::MultiArrayLayout();
  covariance_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  covariance_msg.layout.dim[0].size = this->_slam_solver_->get_covariance().rows();
  covariance_msg.layout.dim[0].stride =
      this->_slam_solver_->get_covariance().rows() * this->_slam_solver_->get_covariance().cols();
  covariance_msg.layout.dim[0].label = "rows";
  covariance_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  covariance_msg.layout.dim[1].size = this->_slam_solver_->get_covariance().cols();
  covariance_msg.layout.dim[1].stride = this->_slam_solver_->get_covariance().cols();
  covariance_msg.layout.dim[1].label = "cols";
  Eigen::MatrixXd covariance_matrix = this->_slam_solver_->get_covariance();
  for (unsigned int i = 0; i < covariance_matrix.rows(); i++) {
    for (unsigned int j = 0; j < covariance_matrix.cols(); j++) {
      covariance_msg.data.push_back(covariance_matrix(i, j));
    }
  }
  this->_covariance_publisher_->publish(covariance_msg);
}

void SLAMNode::_publish_lap_counter() {
  std_msgs::msg::Float64 lap_counter_msg;
  lap_counter_msg.data = this->_slam_solver_->get_lap_counter();
  this->_lap_counter_publisher_->publish(lap_counter_msg);
}

void SLAMNode::_publish_associations() {
  auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  // Add a DELETE_ALL marker to clear previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.frame_id = "map";
  clear_marker.header.stamp = this->now();
  clear_marker.ns = "associations";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array_msg.markers.push_back(clear_marker);

  for (int i = 0; i < this->_associations_.size(); i++) {
    if (this->_associations_[i] >= 0) {
      double observation_x = this->_observations_global_[i * 2];
      double observation_y = this->_observations_global_[i * 2 + 1];
      double map_x = this->_map_coordinates_[this->_associations_[i]];
      double map_y = this->_map_coordinates_[this->_associations_[i] + 1];

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";  // change frame_id if needed
      marker.header.stamp = this->now();
      marker.ns = "associations";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = observation_x;
      start.y = observation_y;
      start.z = 0.0;
      end.x = map_x;
      end.y = map_y;
      end.z = 0.0;

      marker.points.push_back(start);
      marker.points.push_back(end);

      marker.scale.x = 0.15;  // shaft diameter
      marker.scale.y = 0.04;  // head diameter
      marker.scale.z = 0.01;  // head length

      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;

      marker_array_msg.markers.push_back(marker);
    }
  }
  this->_associations_visualization_publisher_->publish(marker_array_msg);
}

/* -------------- Checks if the mission is finished --------*/
bool SLAMNode::_is_mission_finished() const {
  if (this->_vehicle_state_velocities_.velocity_x > 0.1 ||
      this->_vehicle_state_velocities_.velocity_y > 0.1 ||
      this->_vehicle_state_velocities_.rotational_velocity > 0.05) {
    return false;
  }

  if (this->_mission_ == common_lib::competition_logic::Mission::ACCELERATION &&
      this->_vehicle_pose_.position.x > 75.0) {
    return true;
  }
  if (this->_mission_ == common_lib::competition_logic::Mission::SKIDPAD &&
      this->_vehicle_pose_.position.x > 22.0) {
    return true;
  }
  if (this->_mission_ == common_lib::competition_logic::Mission::AUTOCROSS &&
      this->_slam_solver_->get_lap_counter() >= 1) {
    return true;
  }
  if (this->_mission_ == common_lib::competition_logic::Mission::TRACKDRIVE &&
      this->_slam_solver_->get_lap_counter() >= 10) {
    return true;
  }
  return false;
}