#include "adapter_ekf_state_est/eufs.hpp"
#include "ros_node/se_node.hpp"

Adapter::Adapter(SENode *speed_est) {
  this->node = speed_est;

  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}

void Adapter::imu_subscription_callback(const sensor_msgs::msg::Imu msg) {
  double angular_velocity = msg.angular_velocity.z;
  double acceleration_x = msg.linear_acceleration.x;
  double acceleration_y = msg.linear_acceleration.y;

  this->node->_imu_subscription_callback(angular_velocity, acceleration_x, acceleration_y);
}
