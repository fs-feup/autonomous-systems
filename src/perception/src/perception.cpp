#include <cstdio>
#include "perception/perception_node.hpp"
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <perception/perception_node.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto ground_removal = new RANSAC(0.1, 1000);
  auto clustering = new DBSCAN(10, 0.5);
  auto coneDifferentiator = new LeastSquaresDifferentiation();

  auto node = std::make_shared<Perception>(ground_removal, clustering, coneDifferentiator);

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}