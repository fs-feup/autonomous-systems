#include "perception/perception_node.hpp"
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <cstdio>

Perception::Perception(GroundRemoval* groundRemoval) : Node("perception"), groundRemoval(groundRemoval) {
  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&Perception::pointCloudCallback, this, std::placeholders::_1));

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2 msg) {
  
  auto point_cloud = msg.data;

  // Convert byte data to a string
  std::ostringstream ss;
  for (const auto& byte : point_cloud) {
    ss << std::hex << static_cast<int>(byte) << " ";
  }

  //RCLCPP_INFO(this->get_logger(), "PointCloud Data: %s", ss.str().c_str());
  RCLCPP_INFO(this->get_logger(), "Perception is alive!");
}