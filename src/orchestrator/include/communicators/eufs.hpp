#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orchestrator/orchestrator.hpp"

class EufsCommunicator : public Communicator {
 public:
  EufsCommunicator(Orchestrator* orchestrator);
  void send_to_car(const custom_interfaces::msg::VcuCommand msg) override;
  void send_from_car(const nav_msgs::msg::Odometry msg);

 private:
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sim_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sim_subscriber_;

  Orchestrator* orchestrator_;
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_