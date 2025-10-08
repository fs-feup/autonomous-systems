#include "node/mocker_node.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "common_lib/config_load/config_load.hpp"

MockerNode::MockerNode() : rclcpp::Node("mocker_node") {
  auto mocker_config_path =
      common_lib::config_load::get_config_yaml_path("mocker_node", "global", "global_config");
  auto mocker_config = YAML::LoadFile(mocker_config_path);

  auto track_name = mocker_config["global"]["track_name"].as<std::string>();
  auto sim = mocker_config["global"]["adapter"].as<std::string>();

  std::string planning_gtruth_file = std::string(std::filesystem::current_path()) +
                                     "/gtruths/tracks/" + sim + "/" + track_name + "/" +
                                     track_name + "_gtruth.csv";
  std::string se_gtruth_file = std::string(std::filesystem::current_path()) + "/gtruths/tracks/" +
                               sim + "/" + track_name + "/" + track_name + ".csv";

  gtruth_planning = planning_gtruth_fromfile(open_file_as_stream(planning_gtruth_file));
  gtruth_se = se_gtruth_fromfile(open_file_as_stream(se_gtruth_file));

  planning_publisher = this->create_publisher<custom_interfaces::msg::PathPointArray>(
      "/path_planning/mock_path",
      rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
  se_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>(
      "/state_estimation/mock_map",
      rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

  planning_visualization_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
      "/path_planning/smoothed_mock_path", 10);

  this->_map_frame_id_ = sim == "eufs" ? "base_footprint" : "map";

  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&MockerNode::publish_data, this));
}

void MockerNode::publish_data() {
  planning_publisher->publish(gtruth_planning);
  se_publisher->publish(gtruth_se);

  planning_visualization_publisher->publish(
      common_lib::communication::line_marker_from_structure_array(
          common_lib::communication::path_point_array_from_ci_vector(gtruth_planning),
          "mock_path_planning", this->_map_frame_id_, 12, "green"));
}
