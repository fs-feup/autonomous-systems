#include "perception/perception_node.hpp"

#include <cstdio>

Perception::Perception() : Node("perception") {
  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/point_cloud", 10,
      std::bind(&Perception::pointCloudCallback, this, std::placeholders::_1));

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // TODO(davide): do this
}