#pragma once
#include <memory>

#include "custom_interfaces/msg/path_point_array.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "control/include/config/parameters.hpp"
#include "common_lib/structures/control_command.hpp"

/**
 * @brief Base (abstract) class for longitudinal controllers (the ones that calculate throttle)
 */
class LongitudinalController {
protected:
  std::shared_ptr<ControlParameters> params_;
public:
  /**
   * @brief Called when a new path is sent by Path Planning
   */
  virtual void path_callback(const custom_interfaces::msg::PathPointArray& msg) = 0;

  /**
   * @brief Called when the car state (currently just velocity) is updated
   */
  virtual void vehicle_state_callback(const custom_interfaces::msg::Velocities& msg) = 0;

  /**
   * @brief Called when the car pose is updated by SLAM
   */
  virtual void vehicle_pose_callback(const custom_interfaces::msg::Pose& msg) = 0;

  /**
   * @brief Returns the throttle command calculated by the solver (only throttle)
   */
  virtual common_lib::structures::ControlCommand get_throttle_command() = 0;

  /**
   * @brief Publishes solver specific data using the provided ControlNode
   * 
   * @param node shared pointer to the ControlNode
   * @param publisher_map map of topic names to publisher pointers
   */
  virtual void publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) = 0;

  LongitudinalController(const ControlParameters& params) : params_(std::make_shared<ControlParameters>(params)) {};
  virtual ~LongitudinalController() = default;
};
