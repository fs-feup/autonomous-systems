#include "perception/perception_node.hpp"
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../../lib/pcl_conversions/pcl_conversions.h"
#include <cstdio>

Perception::Perception(GroundRemoval* groundRemoval) : Node("perception"), groundRemoval(groundRemoval) {
  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&Perception::pointCloudCallback, this, std::placeholders::_1));

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2 msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //fromPCLPointCloud2(msg, *pcl_cloud);

    RCLCPP_INFO(this->get_logger(), "Perception is alive!");
}