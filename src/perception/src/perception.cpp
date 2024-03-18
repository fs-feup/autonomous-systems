#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>

#include <cstdio>

#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto ground_removal = new RANSAC(0.1, 30);
  auto clustering = new DBSCAN(3, 0.1);
  auto coneDifferentiator = new LeastSquaresDifferentiation();
  auto coneValidator = new CylinderValidator(0.228, 0.325);

  auto node = std::make_shared<Perception>(ground_removal, clustering,
                                           coneDifferentiator, coneValidator);

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}