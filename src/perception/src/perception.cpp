#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>

#include <cone_evaluator/distance_predict.hpp>
#include <cstdio>
#include <memory>
#include <vector>

#include "perception/perception_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("perception_node");

  // Retrieve parameters using the node's interface
  double ransac_epsilon = node->declare_parameter("ransac_epsilon", 0.1);
  int ransac_n_neighbours = node->declare_parameter("ransac_n_neighbours", 30);
  int clustering_n_neighbours = node->declare_parameter("clustering_n_neighbours", 3);
  double clustering_epsilon = node->declare_parameter("clustering_epsilon", 0.1);
  double horizontal_resolution = node->declare_parameter("horizontal_resolution", 0.33);
  double vertical_resolution = node->declare_parameter("vertical_resolution", 0.22);
  std::string mode = node->declare_parameter("adapter", "fsds");

  // Create shared pointers for components
  auto ground_removal = std::make_shared<RANSAC>(ransac_epsilon, ransac_n_neighbours);
  auto clustering = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);
  auto cone_differentiator = std::make_shared<LeastSquaresDifferentiation>();
  std::vector<std::shared_ptr<ConeValidator>> cone_validators = {
      std::make_shared<CylinderValidator>(0.228, 0.325), std::make_shared<HeightValidator>(0.4)};
  auto distance_predict =
      std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);

  // Create perception node with shared pointers
  auto perception_node = std::make_shared<Perception>(
      ground_removal, clustering, cone_differentiator, cone_validators, distance_predict, mode);

  RCLCPP_INFO(node->get_logger(), "Perception is alive! 🚀");

  rclcpp::spin(perception_node);  // Spin the perception node
  rclcpp::shutdown();
  return 0;
}
