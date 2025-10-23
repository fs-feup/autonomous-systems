#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Trait class for node control in SLAM solvers
 * @details This trait provides an interface for SLAM solvers to interact with ROS2 nodes,
 * allowing them to initialize and manage node-related functionalities.
 */
class NodeControllerTrait {
public:
  /**
   * @brief Initialize the SLAM solver
   * @details This method is used to initialize the SLAM solver's
   * aspects that require the node e.g. timer callbacks
   *
   * @param node ROS2 node
   */
  virtual void init(std::weak_ptr<rclcpp::Node> node) = 0;

  virtual ~NodeControllerTrait() = default;
};