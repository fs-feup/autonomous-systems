#include "include/mocker_node.hpp"

MockerNode::MockerNode() : rclcpp::Node("mocker_node") {
  std::string gtruth_file = declare_parameter<std::string>("gtruth_file_path");
  gtruth_planning = gtruth_fromfile(openFileAsStream(gtruth_file));

  // creates publisher for the flag
  finish_publisher = this->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);

  // creates publisher for the planning node
  planning_publisher = this->create_publisher
  <custom_interfaces::msg::PathPointArray>("planning_gtruth", 10);

  // get mission
  mission_signal = this->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&MockerNode::callback, this, std::placeholders::_1));
}


void MockerNode::callback(fs_msgs::msg::GoSignal mission_signal) {
  std::string mission = mission_signal.mission;
  planning_publisher->publish(gtruth_planning);
}
