#include "adapter_perception/fsds.hpp"

#include "perception/perception_node.hpp"

FsdsAdapter::FsdsAdapter(Perception* perception) : Adapter(perception) { this->init(); }

void FsdsAdapter::init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&FsdsAdapter::mission_state_callback, this, std::placeholders::_1));
  this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/Lidar1", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        node->pointCloudCallback(msg);
      });

  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
}

void FsdsAdapter::mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  std::cout << "Mission State undefined for Fsds\n";
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true;  // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}
