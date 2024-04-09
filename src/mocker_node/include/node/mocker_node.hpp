#ifndef MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP
#define MOCKER_NODE_INCLUDE_MOCKER_NODE_HPP

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "planning/planning_mock.hpp"

class MockerNode : public rclcpp::Node {
 private:
  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr finish_publisher;
  rclcpp::Publisher<custom_interfaces::msg::PathPointArray>::SharedPtr planning_publisher;
  rclcpp::Subscription<fs_msgs::msg::GoSignal>::SharedPtr mission_signal;

 public:
    custom_interfaces::msg::PathPointArray gtruth_planning; // ground truth for the planning node

    /**
     * @brief publish the messages
    */
    void callback(fs_msgs::msg::GoSignal mission_signal);

    /**
     * @brief Construct a new Mocker Node object
     * 
     * @param gtruth_file_path path to the file where the planning ground truth is
     * 
     */
    explicit MockerNode(std::string gtruth_file_path);

    /**
     * @brief Default constructer for a new Mocker Node object
     * 
     */
    MockerNode();
};

#endif