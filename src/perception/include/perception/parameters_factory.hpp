#pragma once

#include <string>
#include <vector>

#include "perception/perception_node.hpp"

struct PerceptionParameters;

PerceptionParameters load_adapter_parameters() {
  PerceptionParameters params;

  auto adapter_node = std::make_shared<rclcpp::Node>("perception_adapter");

  double ransac_epsilon = adapter_node->declare_parameter("ransac_epsilon", 0.05);
  int ransac_n_neighbours = adapter_node->declare_parameter("ransac_n_neighbours", 20);
  int clustering_n_neighbours = adapter_node->declare_parameter("clustering_n_neighbours", 1);
  double clustering_epsilon = adapter_node->declare_parameter("clustering_epsilon", 0.1);
  double horizontal_resolution = adapter_node->declare_parameter("horizontal_resolution", 0.33);
  double vertical_resolution = adapter_node->declare_parameter("vertical_resolution", 0.22);
  std::string ground_removal_algoritm =
      adapter_node->declare_parameter("ground_removal", "grid_ransac");
  std::string target_file = adapter_node->declare_parameter("target_file", "cone.pcd");
  double max_correspondence_distance =
      adapter_node->declare_parameter("max_correspondence_distance", 0.1);
  int max_iteration = adapter_node->declare_parameter("max_iteration", 100);
  double transformation_epsilon = adapter_node->declare_parameter("transformation_epsilon", 1e-8);
  double euclidean_fitness_epsilon =
      adapter_node->declare_parameter("euclidean_fitness_epsilon", 1e-6);
  double fov_trim = adapter_node->declare_parameter("fov_trim", 90);
  params.adapter_ = adapter_node->declare_parameter("adapter", "fsds");
  params.pc_max_range_ = adapter_node->declare_parameter("pc_max_range", 15.0);

  // Create shared pointers for components
  if (ground_removal_algoritm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_n_neighbours);
  } else if (ground_removal_algoritm == "grid_ransac") {
    int n_angular_grids = adapter_node->declare_parameter("n_angular_grids", 7);
    double radius_resolution = adapter_node->declare_parameter("radius_resolution", 7.5);
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_n_neighbours,
                                                          n_angular_grids, radius_resolution);
  }

  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);
  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();
  params.cone_validators_ = {std::make_shared<CylinderValidator>(0.228, 0.325),
                             std::make_shared<HeightValidator>(0.325)};
  params.distance_predict_ =
      std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);

  params.icp_ = std::make_shared<ICP>(target_file, max_correspondence_distance, max_iteration,
                                      transformation_epsilon, euclidean_fitness_epsilon);
  params.fov_trim_ = fov_trim;

  return params;
}
