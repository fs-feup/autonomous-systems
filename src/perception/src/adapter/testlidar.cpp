#include "adapter/testlidar.hpp"
#include "perception/perception_node.hpp"

TestAdapter::TestAdapter(Perception* perception) : Adapter(perception) {
  this->init();
}

void TestAdapter::init() {
   this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {node->pointCloudCallback(msg);});
}

void TestAdapter::mission_state_callback(const std::string msg) {
  // Test Class, No implementation for mission state
}

void TestAdapter::finish() {
  // Test Class, No implementation for finish
}



