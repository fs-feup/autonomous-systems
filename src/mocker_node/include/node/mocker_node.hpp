#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "common_lib/communication/marker.hpp"
#include "common_lib/communication/planning.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/mocks.hpp"
#include "visualization_msgs/msg/marker.hpp"

class MockerNode : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr planning_publisher;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr se_publisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planning_visualization_publisher;

  /**< Timer for the periodic publishing */
  rclcpp::TimerBase::SharedPtr timer_;

 public:
  custom_interfaces::msg::PathPointArray gtruth_planning;  // ground truth for the planning node
  custom_interfaces::msg::ConeArray gtruth_se;  // ground truth for the state estimation node

  /**
   * @brief publish the messages
   */
  void publish_data();

  /**
   * @brief Constructor for a new Mocker Node object
   * @param track_name desired mock track_name file / folder
   * @param sim desired simulator that contains a track
   */
  MockerNode(const std::string &track_name, const std::string &sim);
};
