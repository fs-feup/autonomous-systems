#include "perception/perception_node.hpp"
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstdio>


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto ground_removal = new RANSAC(1.1, 5);

  auto node = std::make_shared<Perception>(ground_removal);

   pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclCloud->width = 5;
    pclCloud->height = 1;
    pclCloud->points.resize(pclCloud->width * pclCloud->height);

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}