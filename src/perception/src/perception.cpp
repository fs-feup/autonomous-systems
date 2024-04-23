#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <cstdio>
#include <cone_evaluator/distance_predict.hpp>
#include <vector>
#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto ground_removal = new RANSAC(0.1, 30);
  auto clustering = new DBSCAN(3, 0.1);
  auto coneDifferentiator = new LeastSquaresDifferentiation();
  std::vector<ConeValidator*> coneValidator = {new CylinderValidator(0.228, 0.325),
                                               new HeightValidator(0.325)};

  auto distancePredict = new DistancePredict(0.33, 0.2);

  auto node =
      std::make_shared<Perception>(ground_removal, clustering, coneDifferentiator,
                                   coneValidator, distancePredict);

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}