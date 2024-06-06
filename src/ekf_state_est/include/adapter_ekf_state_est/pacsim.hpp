#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <std_srvs/srv/empty.hpp>

#include "pacsim/msg/perception_detections.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "ros_node/se_node.hpp"

class PacsimAdapter : public SENode {
  message_filters::Subscriber<pacsim::msg::Wheels>
      _pacsim_wheel_speeds_subscription_;  ///< Subscriber for wheel speeds
  message_filters::Subscriber<pacsim::msg::StampedScalar>
      _steering_angle_subscription_;  ///< Subscriber for steering angle

  using WheelSteerPolicy = message_filters::sync_policies::ApproximateTime<
      pacsim::msg::Wheels,
      pacsim::msg::StampedScalar>;  ///< Policy for synchronizing wheel speeds and steering angle
  std::shared_ptr<message_filters::Synchronizer<WheelSteerPolicy>>
      _sync_;  ///< Synchronizer for wheel speeds and steering angle

  rclcpp::Subscription<pacsim::msg::PerceptionDetections>::SharedPtr
      _perception_detections_subscription_;  ///< Subscriber for simulated perception detections

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr
      _finished_client_;  ///< Client for finished signal

  /**
   * @brief Wheel speed subscription callback, which receives both wheel speeds and steering angle
   * through a synchronizer message filter
   */
  void wheel_speeds_subscription_callback(const pacsim::msg::Wheels& wheels_msg,
                                          const pacsim::msg::StampedScalar& steering_angle_msg);

  /**
   * @brief Callback for simulated perception detections from pacsim
   *
   * @param msg Message containing the array of perceived detections
   */
  void perception_detections_subscription_callback(const pacsim::msg::PerceptionDetections& msg);

public:
  explicit PacsimAdapter(bool use_odometry, bool use_simulated_perception,
                         std::string motion_model_name, std::string data_assocation_model_name,
                         float sml_da_curvature, float sml_initial_limit, float observation_noise,
                         float wheel_speed_sensor_noise, float data_association_limit_distance);

  void finish() final;
};