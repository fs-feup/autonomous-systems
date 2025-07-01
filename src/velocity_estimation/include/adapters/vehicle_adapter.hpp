#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "custom_interfaces/msg/imu_data.hpp"
#include "custom_interfaces/msg/operational_status.hpp"
#include "custom_interfaces/msg/steering_angle.hpp"
#include "custom_interfaces/msg/wheel_rpm.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "node/node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

/**
 * @brief Adapter class for the vehicle.
 *
 * This class subscribes to the IMU, wheel speed sensors, and steering angle topics published by the
 * vehicle. It then passes the data to the velocity estimator for processing.
 */
class VehicleAdapter : public VENode {
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
      custom_interfaces::msg::WheelRPM>;  ///< Policy for synchronizing wheel speeds
  std::shared_ptr<message_filters::Synchronizer<WheelSSPolicy>>
      _wss_sync_;  ///< Synchronizer for wheel speeds
  std::shared_ptr<message_filters::Synchronizer<XsensImuPolicy>> _xsens_imu_sync_;

  rclcpp::Subscription<custom_interfaces::msg::SteeringAngle>::SharedPtr _steering_angle_sub_;

  rclcpp::Subscription<custom_interfaces::msg::WheelRPM>::SharedPtr _resolver_sub_;

  double last_decent_fl_wss_reading = 0;

public:
  explicit VehicleAdapter(const VEParameters& parameters);
  /**
   * @brief IMU subscription callback, which receives both free acceleration and angular velocity
   * through a synchronizer message filter
   */
  void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr& free_acceleration_msg,
                    const geometry_msgs::msg::Vector3Stamped::SharedPtr& angular_velocity_msg);
  /**
   * @brief Wheel speed subscription callback, which receives both wheel speeds
   * through a synchronizer message filter
   */
  void wss_callback(const custom_interfaces::msg::WheelRPM& rl_wheel_rpm_msg,
                    const custom_interfaces::msg::WheelRPM& rr_wheel_rpm_msg);

  /**
   * @brief Callback for the subscription of the steering angle sensor
   */
  void steering_angle_callback(const custom_interfaces::msg::SteeringAngle msg);

  /**
   * @brief Callback for the subscription of the resolver which measures the motor RPM
   */
  void resolver_callback(custom_interfaces::msg::WheelRPM msg);
};