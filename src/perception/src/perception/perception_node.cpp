#include "perception/perception_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>

Perception::Perception(GroundRemoval* groundRemoval) : Node("perception"),
        groundRemoval(groundRemoval) {
  this->adapter = new Adapter("fsds", this);

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *pcl_cloud);


    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);


    groundRemoval->groundRemoval(pcl_cloud, ground_removed_cloud);

    for (const auto& point : ground_removed_cloud->points) {
        RCLCPP_INFO(this->get_logger(), "Point: x=%f, y=%f, z=%f, intensity=%f",
                    point.x, point.y, point.z, point.intensity);
    }
}