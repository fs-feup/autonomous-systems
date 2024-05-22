#include "adapter_perception/eufs.hpp"

#include "perception/perception_node.hpp"

EufsAdapter::EufsAdapter(Perception* perception) : Adapter(perception) {
  this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        node->pointCloudCallback(msg);
      });

  this->eufs_mission_state_client_ =
      this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/set_mission");

  this->eufs_ebs_client_ = this->node->create_client<eufs_msgs::srv::SetCanState>("/ros_can/ebs");
}


void EufsAdapter::finish() {
  std::cout << "Finish undefined for Eufs\n";
}
