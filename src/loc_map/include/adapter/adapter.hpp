#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "loc_map/data_structures.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class LMSubscriber;

class Adapter {
  LMSubscriber* node;
  Mission mission = Mission::acceleration;

  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

  void eufs_init();
  void fsds_init();
  void ads_dv_init();

  void eufs_mission_state_callback(const eufs_msgs::msg::CanState msg);
  void eufs_imu_subscription_callback(const sensor_msgs::msg::Imu msg);
  void eufs_wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped msg);

  void eufs_set_mission_state(int mission, int state);

 public:
  Adapter(std::string mode, LMSubscriber* subscriber);

  void set_mission(Mission mission);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
