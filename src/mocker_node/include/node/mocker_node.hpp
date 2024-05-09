#ifndef MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP
#define MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "planning/planning_mock.hpp"
#include "rclcpp/rclcpp.hpp"

class MockerNode : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr planning_publisher;

 public:
  custom_interfaces::msg::PathPointArray gtruth_planning;  // ground truth for the planning node

  /**
   * @brief publish the messages
   */
  void callback();

  /**
   * @brief Default constructer for a new Mocker Node object
   *
   */
  MockerNode();
};

#endif