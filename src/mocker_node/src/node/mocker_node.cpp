#include "node/mocker_node.hpp"

#include <filesystem>

MockerNode::MockerNode() : rclcpp::Node("mocker_node") {
  std::string planning_gtruth_default_file_path =
      std::string(std::filesystem::current_path()) +
      "/gtruths/tracks_gtruths/track1/track1_gtruth.csv";
  std::string se_gtruth_default_file_path =
      std::string(std::filesystem::current_path()) + "/gtruths/tracks_gtruths/track1/track1.csv";
  std::string planning_gtruth_file = declare_parameter<std::string>(
      "planning_gtruth_file_path", planning_gtruth_default_file_path);
  std::string se_gtruth_file =
      declare_parameter<std::string>("se_gtruth_file_path", se_gtruth_default_file_path);
  gtruth_planning = planning_gtruth_fromfile(open_file_as_stream(planning_gtruth_file));
  gtruth_se = se_gtruth_fromfile(open_file_as_stream(se_gtruth_file));

  planning_publisher = this->create_publisher<custom_interfaces::msg::PathPointArray>(
      "planning_gtruth", rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
  se_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>(
      "track_map", rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&MockerNode::publish_data, this));
}

void MockerNode::publish_data() {
  planning_publisher->publish(gtruth_planning);
  se_publisher->publish(gtruth_se);
}
