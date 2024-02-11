#include "perception/perception_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>

Perception::Perception(GroundRemoval* groundRemoval, Clustering* clustering) : Node("perception"),
        groundRemoval(groundRemoval), clustering(clustering) {
  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&Perception::pointCloudCallback, this, std::placeholders::_1));

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *pcl_cloud);


    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    groundRemoval->groundRemoval(pcl_cloud, ground_removed_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    clustering->clustering(ground_removed_cloud, &clusters);

    RCLCPP_INFO(this->get_logger(), "---------- Point Cloud Received ----------");
    RCLCPP_INFO(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
        pcl_cloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
        ground_removed_cloud->points.size());
    RCLCPP_INFO(this->get_logger(), "Point Cloud after Clustering: %ld clusters",
        clusters.size());
}