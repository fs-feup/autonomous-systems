#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/odom.hpp"
#include "custom_interfaces/msg/state.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orchestrator/orchestrator.hpp"

class EufsCommunicator : public Communicator {
 public:
  EufsCommunicator(Orchestrator* orchestrator);
  void send_to_car(const custom_interfaces::msg::VcuCommand msg) override;

 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sim_publisher_;
  rclcpp::Subscription<custom_interfaces::msg::State>::SharedPtr state_subscriber_;
  rclcpp::Subscription<custom_interfaces::msg::Odom>::SharedPtr odom_subscriber_;

  void send_state_from_car(const custom_interfaces::msg::State msg);
  void send_odom_from_car(const custom_interfaces::msg::Odom msg);

  Orchestrator* orchestrator_;
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_