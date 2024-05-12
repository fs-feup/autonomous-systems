#ifndef MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP
#define MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "utils/mocks.hpp"
#include "rclcpp/rclcpp.hpp"

class MockerNode : public rclcpp::Node {
 private:
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr planning_publisher;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr se_publisher;

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
   * @brief Default constructer for a new Mocker Node object
   *
   */
  MockerNode();
};

#endif