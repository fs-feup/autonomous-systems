#include "adapter_ekf_state_est/eufs.hpp"
#include "ros_node/se_node.hpp"

Adapter::Adapter(std::shared_ptr<SENode> se_node) : node(se_node) {
  RCLCPP_INFO(this->node->get_logger(), "Adapter created");
}

void Adapter::imu_subscription_callback(const sensor_msgs::msg::Imu& msg) {
  this->node->_imu_subscription_callback(msg);
}
