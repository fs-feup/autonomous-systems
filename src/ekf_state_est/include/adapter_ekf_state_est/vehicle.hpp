#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "adapter_ekf_state_est/adapter.hpp"
#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

class SENode;

class VehicleAdapter : public Adapter {
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM>
      _rl_wheel_rpm_subscription_;  ///< Subscriber for rl wheel rpm
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM>
      _rr_wheel_rpm_subscription_;  ///< Subscriber for rr wheel rpm
  message_filters::Subscriber<custom_interfaces::msg::SteeringAngle>
      _steering_angle_subscription_;  ///< Subscriber for steering angle

  message_filters::Subscriber<custom_interfaces::msg::ImuData> _roll_accx_imu_subscription_;
  message_filters::Subscriber<custom_interfaces::msg::ImuData> _yaw_accy_imu_subscription_;

  using ImuPolicy = message_filters::sync_policies::ApproximateTime<
      custom_interfaces::msg::ImuData,
      custom_interfaces::msg::ImuData>;  ///< Policy for synchronizing IMU data
  using WheelSteerPolicy = message_filters::sync_policies::ApproximateTime<
      custom_interfaces::msg::WheelRPM, custom_interfaces::msg::WheelRPM,
      custom_interfaces::msg::SteeringAngle>;  ///< Policy for synchronizing wheel speeds and
                                               ///< steering angle
  std::shared_ptr<message_filters::Synchronizer<WheelSteerPolicy>>
      _sync_;  ///< Synchronizer for wheel speeds and steering angle

  std::shared_ptr<message_filters::Synchronizer<ImuPolicy>> _imu_sync_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
      _finished_client_;  ///< Client for finished signal

  /**
   * @brief Wheel speed subscription callback, which receives both wheel speeds and steering angle
   * through a synchronizer message filter
   */
  void wheel_speeds_subscription_callback(
      const custom_interfaces::msg::WheelRPM& rl_wheel_rpm_msg,
      const custom_interfaces::msg::WheelRPM& rr_wheel_rpm_msg,
      const custom_interfaces::msg::SteeringAngle& steering_angle_msg);

  /**
   * @brief IMU subscriptio callback, which receives both roll and yaw acceleration data
   * separately and synchronizes them
   *
   * @param roll_accx_data roll and acceleration in x axis data
   * @param yaw_accy_data yaw and acceleration in y axis data
   */
  void imu_subscription_callback(const custom_interfaces::msg::ImuData& roll_accx_data,
                                 const custom_interfaces::msg::ImuData& yaw_accy_data);

public:
  explicit VehicleAdapter(std::shared_ptr<SENode> se_node);

  void init() final;
  void finish() final;
};