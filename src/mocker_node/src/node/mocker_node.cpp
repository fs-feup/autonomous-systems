#include "node/mocker_node.hpp"

#include <filesystem>

MockerNode::MockerNode() : rclcpp::Node("mocker_node") {
  std::string gtruth_default_file_path = std::string(std::filesystem::current_path()) +
                                         "/gtruths/tracks_gtruths/skidpad/skidpad_gtruth.csv";
  std::string gtruth_file =
      declare_parameter<std::string>("gtruth_file_path", gtruth_default_file_path);
  gtruth_planning = gtruth_fromfile(openFileAsStream(gtruth_file));

  planning_publisher = this->create_publisher<custom_interfaces::msg::PathPointArray>(
      "planning_gtruth", rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

  callback();
}

void MockerNode::callback() { planning_publisher->publish(gtruth_planning); }
