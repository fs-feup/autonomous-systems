#include "adapter_perception/vehicle.hpp"
#include "perception/perception_node.hpp"

VehicleAdapter::VehicleAdapter(const PerceptionParameters& params) : Perception(params) {
  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/hesai/pandar", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointCloudCallback(msg);
      });
}

void VehicleAdapter::mission_state_callback(const custom_interfaces::msg::OperationalStatus& msg) {
  // Test Class, No implementation for mission state
}

void VehicleAdapter::finish() {
  // Test Class, No implementation for finish
}
