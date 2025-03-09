#include "ros_node/slam_node.hpp"

#include <fstream>

#include "common_lib/communication/marker.hpp"
#include "common_lib/maths/transformations.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "motion_lib/v2p_models/map.hpp"
#include "perception_sensor_lib/data_association/map.hpp"
#include "slam_solver/map.hpp"

/*---------------------- Constructor --------------------*/

SLAMNode::SLAMNode(const SLAMParameters &params) : Node("slam") {
  // Initialize the models
  std::shared_ptr<V2PMotionModel> motion_model = v2p_models_map.at(params.motion_model_name_)();
  std::shared_ptr<DataAssociationModel> data_association =
      data_association_models_map.at(params.data_association_model_name_)(DataAssociationParameters(
          params.data_association_limit_distance_, params.data_association_gate_,
          params.new_landmark_confidence_gate_, params.observation_x_noise_,
          params.observation_y_noise_));

  // Initialize SLAM solver object
  this->_slam_solver_ = slam_solver_constructors_map.at(params.slam_solver_name_)(
      params, data_association, motion_model);

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
  this->_position_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/state_estimation/visualization/position", 10);
  this->_correction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/correction_step", 10);
  this->_prediction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/prediction_step", 10);
}

/*---------------------- Subscriptions --------------------*/

void SLAMNode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;

  if (!this->_go_) {
    return;
  }

  rclcpp::Time start_time = this->get_clock()->now();

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

  rclcpp::Time end_time = this->get_clock()->now();

  // Execution Time calculation
  std_msgs::msg::Float64 correction_execution_time;
  correction_execution_time.data = (end_time - start_time).seconds() * 1000.0;
  this->_correction_execution_time_publisher_->publish(correction_execution_time);
  this->_publish_vehicle_pose();
  this->_publish_map();
}

void SLAMNode::_velocities_subscription_callback(const custom_interfaces::msg::Velocities &msg) {
  this->_vehicle_state_velocities_ = common_lib::structures::Velocities(
      msg.velocity_x, msg.velocity_y, msg.angular_velocity, msg.covariance[0], msg.covariance[4],
      msg.covariance[8], msg.header.stamp);
  this->_slam_solver_->add_motion_prior(this->_vehicle_state_velocities_);
  this->_vehicle_pose_ = this->_slam_solver_->get_pose_estimate();
  this->_publish_vehicle_pose();
}

/*---------------------- Publications --------------------*/

void SLAMNode::_publish_vehicle_pose() {
  auto message = custom_interfaces::msg::Pose();
  message.x = this->_vehicle_pose_.position.x;
  message.y = this->_vehicle_pose_.position.y;
  message.theta = this->_vehicle_pose_.orientation;
  // TODO(marhcouto): add covariance
  message.header.stamp = this->get_clock()->now();

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f)", message.x, message.y, message.theta);
  this->_vehicle_pose_publisher_->publish(message);
}

void SLAMNode::_publish_map() {
  auto cone_array_msg = custom_interfaces::msg::ConeArray();
  auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (common_lib::structures::Cone const &cone : this->_track_map_) {
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position.x = cone.position.x;
    cone_message.position.y = cone.position.y;
    // TODO(marhcouto): add covariance
    cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
    cone_array_msg.cone_array.push_back(cone_message);
    RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
                 cone_message.position.y, cone_message.color.c_str());
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  cone_array_msg.header.stamp = this->get_clock()->now();
  this->_map_publisher_->publish(cone_array_msg);
  marker_array_msg = common_lib::communication::marker_array_from_structure_array(
      this->_track_map_, "map_cones", _adapter_name_ == "eufs" ? "base_footprint" : "map");
  this->_visualization_map_publisher_->publish(marker_array_msg);
}
