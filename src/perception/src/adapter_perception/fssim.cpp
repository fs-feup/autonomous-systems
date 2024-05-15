#include "adapter_perception/fssim.hpp"

#include "perception/perception_node.hpp"

FSSimAdapter::FSSimAdapter(Perception* perception) : Adapter(perception) {}

void FSSimAdapter::init() {
  this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "lidar/raw", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        node->pointCloudCallback(msg);
      });

}


void FSSimAdapter::finish() {
  std::cout << "Finish undefined for FSSim\n";
}
