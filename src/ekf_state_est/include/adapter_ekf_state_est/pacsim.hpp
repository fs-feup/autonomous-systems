#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <std_srvs/srv/empty.hpp>

#include "adapter_ekf_state_est/adapter.hpp"
#include "pacsim/msg/perception_detections.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"

class SENode;

class PacsimAdapter : public Adapter {
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
  explicit PacsimAdapter(std::shared_ptr<SENode> se_node);

  void init() final;
  void finish() final;
};