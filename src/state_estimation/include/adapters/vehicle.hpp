#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "common_lib/structures/control_command.hpp"
#include "custom_interfaces/msg/control_command.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "node/node.hpp"

/**
 * @brief Adapter class for the real vehicle (FSFEUP02).
 *
 * Subscribes to the car's IMU, front wheel-speed sensors, steering-angle sensor and motor
 * resolver, and forwards the data to the state estimator. The car is RWD with no rear
 * wheel-speed sensors, so the rear axle is observed through the motor RPM. Publishing is
 * driven by the SENode timer, so the callbacks only feed the estimator.
 */
class VehicleAdapter : public SENode {
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM>
      _fl_wheel_rpm_subscription_;  ///< Subscriber for fl wheel rpm
  message_filters::Subscriber<custom_interfaces::msg::WheelRPM>
      _fr_wheel_rpm_subscription_;  ///< Subscriber for fr wheel rpm

  message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> _free_acceleration_subscription_;
  message_filters::Subscriber<geometry_msgs::msg::Vector3Stamped> _angular_velocity_subscription_;
  using XsensImuPolicy = message_filters::sync_policies::ApproximateTime<
      geometry_msgs::msg::Vector3Stamped,
      geometry_msgs::msg::Vector3Stamped>;  ///< Policy for synchronizing Xsens IMU data
  using WheelSSPolicy = message_filters::sync_policies::ApproximateTime<
      custom_interfaces::msg::WheelRPM,
      custom_interfaces::msg::WheelRPM>;  ///< Policy for synchronizing front wheel speeds
  std::shared_ptr<message_filters::Synchronizer<WheelSSPolicy>> _wss_sync_;
  std::shared_ptr<message_filters::Synchronizer<XsensImuPolicy>> _xsens_imu_sync_;

  rclcpp::Subscription<custom_interfaces::msg::SteeringAngle>::SharedPtr _steering_angle_sub_;
  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr _resolver_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ControlCommand>::SharedPtr _control_sub_;

  double average_imu_bias_ = 0.0;
  int number_of_imu_readings_ = 0;

public:
  explicit VehicleAdapter(const std::shared_ptr<SEParameters>& parameters);

  /**
   * @brief IMU callback receiving free acceleration and angular velocity through a synchronizer.
   * The first readings are used to estimate the yaw-rate bias before any data is forwarded.
   */
  void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
                    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg);

  /**
   * @brief Front wheel-speed callback receiving FL and FR rpm through a synchronizer.
   */
  void wss_callback(const custom_interfaces::msg::WheelRPM& fl_wheel_rpm_msg,
                    const custom_interfaces::msg::WheelRPM& fr_wheel_rpm_msg);

  /**
   * @brief Callback for the steering-angle sensor.
   */
  void steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg);

  /**
   * @brief Callback for the resolver, which measures the motor RPM (rear axle, RWD).
   */
  void resolver_callback(custom_interfaces::msg::WheelRPM msg);

  /**
   * @brief Callback for the control output published by the control package, used by the
   * process model's prediction step.
   */
  void control_callback(const custom_interfaces::msg::ControlCommand msg);
};
