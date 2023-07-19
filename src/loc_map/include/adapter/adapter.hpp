#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "loc_map/data_structures.hpp"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class LMNode;

class Adapter {
  LMNode* node;
  Mission mission = Mission::acceleration;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _eufs_imu_subscription;
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr _eufs_mission_state_subscription;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr
      _eufs_wheel_speeds_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _fs_imu_subscription;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr _fs_wheel_speeds_subscription;

  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  void eufs_init();
  void fsds_init();
  void ads_dv_init();

  void imu_subscription_callback(const sensor_msgs::msg::Imu msg);
  void eufs_mission_state_callback(const eufs_msgs::msg::CanState msg);
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal);
  void eufs_wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped msg);
  void fsds_wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates msg);

  void eufs_set_mission_state(int mission, int state);

 public:
  Adapter(std::string mode, LMNode* subscriber);

  void set_mission(Mission mission);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
