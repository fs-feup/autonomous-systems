#include "adapter_planning/eufs.hpp"

#include "custom_interfaces/msg/pose.hpp"
#include "planning/planning.hpp"

visualization_msgs::msg::Marker marker_from_cone_w_covariance(
    int id, const std::array<float, 4>& color_array, const eufs_msgs::msg::ConeWithCovariance& cone,
    std::string name_space = "recieved_path_from_eufs", std::string frame_id = "map",
    std::string shape = "cylinder", float scale = 0.5,
    int action = visualization_msgs::msg::Marker::MODIFY) {
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = name_space;
  marker.id = id;
  marker.type = common_lib::communication::marker_shape_map.at(shape);
  marker.action = action;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.pose.position.x = cone.point.x;
  marker.pose.position.y = cone.point.y;
  marker.pose.position.z = 0;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = color_array[0];
  marker.color.g = color_array[1];
  marker.color.b = color_array[2];
  marker.color.a = color_array[3];

  marker.lifetime = rclcpp::Duration(std::chrono::duration<double>(5));
  return marker;
}

visualization_msgs::msg::MarkerArray marker_array_from_cone_array_w_covariance(
    const eufs_msgs::msg::ConeArrayWithCovariance& cone_array,
    std::string name_space = "recieved_path_from_eufs", std::string frame_id = "map",
    std::string shape = "cylinder", float scale = 0.5,
    int action = visualization_msgs::msg::Marker::MODIFY) {
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  auto color_map = common_lib::communication::marker_color_map.at("blue");
  for (auto& c : cone_array.blue_cones) {
    auto marker =
        marker_from_cone_w_covariance(id, color_map, c, name_space, frame_id, shape, scale, action);
    marker_array.markers.push_back(marker);
    id++;
  }

  color_map = common_lib::communication::marker_color_map.at("yellow");
  for (auto& c : cone_array.yellow_cones) {
    auto marker =
        marker_from_cone_w_covariance(id, color_map, c, name_space, frame_id, shape, scale, action);
    marker_array.markers.push_back(marker);
    id++;
  }

  color_map = common_lib::communication::marker_color_map.at("orange");
  for (auto& c : cone_array.orange_cones) {
    auto marker =
        marker_from_cone_w_covariance(id, color_map, c, name_space, frame_id, shape, scale, action);
    marker_array.markers.push_back(marker);
    id++;
  }
  for (auto& c : cone_array.big_orange_cones) {
    auto marker =
        marker_from_cone_w_covariance(id, color_map, c, name_space, frame_id, shape, scale, action);
    marker_array.markers.push_back(marker);
    id++;
  }

  return marker_array;
}

EufsAdapter::EufsAdapter(const PlanningParameters& params) : Planning(params) {
  if (this->planning_config_.simulation_.using_simulated_se_) {
    RCLCPP_INFO(this->get_logger(), "EUFS using simulated State Estimation");
    this->eufs_pose_subscription_ = this->create_subscription<eufs_msgs::msg::CarState>(
        "/odometry_integration/car_state", 10,
        std::bind(&EufsAdapter::pose_callback, this, std::placeholders::_1));
  }
  this->eufs_map_subscription_ = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      "/ground_truth/track", 10,
      std::bind(&EufsAdapter::map_callback, this, std::placeholders::_1));

  this->eufs_state_subscription_ = this->create_subscription<eufs_msgs::msg::CanState>(
      "/ros_can/state", 10,
      std::bind(&EufsAdapter::mission_state_callback, this, std::placeholders::_1));

  this->eufs_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/path_planning/recieved_eufs_map", 10);

  this->eufs_mission_state_client_ =
      this->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");
  RCLCPP_INFO(this->get_logger(), "Planning: EUFS adapter created");
}

void EufsAdapter::mission_state_callback(eufs_msgs::msg::CanState msg) {
  auto mission = msg.ami_state;
  // map eufs mission to system mission
  this->set_mission(
      common_lib::competition_logic::eufs_to_system.at(static_cast<uint16_t>(mission)));
}

void EufsAdapter::set_mission_state(int mission, int state) {
  auto request = std::make_shared<eufs_msgs::srv::SetCanState::Request>();
  request->ami_state = mission;
  request->as_state = state;

  auto result_future = this->eufs_mission_state_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Planning : failed to call mission service");
  }
}

void EufsAdapter::pose_callback(const eufs_msgs::msg::CarState& msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received pose from EUFS");
  custom_interfaces::msg::Pose pose;
  // only gets the x, y, and theta since those are the only ones necessary for planning
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = atan2(2.0f * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                             msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                     msg.pose.pose.orientation.w * msg.pose.pose.orientation.w +
                         msg.pose.pose.orientation.x * msg.pose.pose.orientation.x -
                         msg.pose.pose.orientation.y * msg.pose.pose.orientation.y -
                         msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
  this->vehicle_localization_callback(pose);
}

void EufsAdapter::finish() { RCLCPP_DEBUG(this->get_logger(), "Finish undefined for Eufs\n"); }

void EufsAdapter::map_callback(const eufs_msgs::msg::ConeArrayWithCovariance& msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received cones from EUFS");
  this->eufs_map_publisher_->publish(marker_array_from_cone_array_w_covariance(
      msg, "recieved_path_from_eufs", this->_map_frame_id_));
  if (!this->planning_config_.simulation_.using_simulated_se_) {
    return;
  }
  custom_interfaces::msg::ConeArray cones;
  for (auto cone : msg.blue_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.yellow_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.orange_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.big_orange_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  for (auto cone : msg.unknown_color_cones) {
    custom_interfaces::msg::Cone c;
    c.position.x = cone.point.x;
    c.position.y = cone.point.y;
    cones.cone_array.push_back(c);
  }
  this->track_map_callback(cones);
}
