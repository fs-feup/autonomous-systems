#include "adapter_ekf_state_est/eufs.hpp"
#include "ros_node/se_node.hpp"

Adapter::Adapter(std::shared_ptr<SENode> se_node) : node_(se_node) {
  RCLCPP_INFO(this->node_->get_logger(), "Adapter created");
}

void Adapter::imu_subscription_callback(const sensor_msgs::msg::Imu& msg) const {
  this->node_->_imu_subscription_callback(msg);
}
