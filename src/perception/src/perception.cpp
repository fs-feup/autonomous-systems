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
  double ransac_epsilon = node->declare_parameter("ransac_epsilon", 0.05);
  int ransac_n_neighbours = node->declare_parameter("ransac_n_neighbours", 10);
  int clustering_n_neighbours = node->declare_parameter("clustering_n_neighbours", 1);
  double clustering_epsilon = node->declare_parameter("clustering_epsilon", 1.0);
  double horizontal_resolution = node->declare_parameter("horizontal_resolution", 0.2);
  double vertical_resolution = node->declare_parameter("vertical_resolution", 0.33);
  std::string mode = node->declare_parameter("adapter", "vehicle");
  std::string ground_removal_algorithm = node->declare_parameter("ground_removal", "ransac");
  int n_angular_grids = node->declare_parameter("n_angular_grids", 6);
  double radius_resolution = node->declare_parameter("radius_resolution", 10.0);
  std::string target_file = "src/perception/src/icp/cones/" + node->declare_parameter("target_file", "cone.pcd");
  double max_correspondence_distance = node->declare_parameter("max_correspondence_distance", 0.1);
  int max_iteration = node->declare_parameter("max_iteration", 100);
  double transformation_epsilon = node->declare_parameter("transformation_epsilon", 1e-8);
  double euclidean_fitness_epsilon = node->declare_parameter("euclidean_fitness_epsilon", 1e-6);

  std::shared_ptr<GroundRemoval> ground_removal;

  // Create shared pointers for components
  if (ground_removal_algorithm == "ransac") {
    ground_removal = std::make_shared<RANSAC>(ransac_epsilon, ransac_n_neighbours);
  } else if (ground_removal_algorithm == "grid_ransac") {
    ground_removal = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_n_neighbours, n_angular_grids, radius_resolution);
  }

  auto clustering = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);
  auto cone_differentiator = std::make_shared<LeastSquaresDifferentiation>();
  std::vector<std::shared_ptr<ConeValidator>> cone_validators = {
      std::make_shared<CylinderValidator>(0.228, 0.325), std::make_shared<HeightValidator>(0.4)};
  auto distance_predict = std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);
  auto icp = std::make_shared<ICP>(target_file, max_correspondence_distance, max_iteration, transformation_epsilon, euclidean_fitness_epsilon);

  // Create perception node with shared pointers
  auto perception_node = std::make_shared<Perception>(
      ground_removal, clustering, cone_differentiator, cone_validators, distance_predict, mode, icp);

  RCLCPP_INFO(node->get_logger(), "Perception is alive! 🚀");

  rclcpp::spin(perception_node);  // Spin the perception node
  rclcpp::shutdown();
  return 0;
}
