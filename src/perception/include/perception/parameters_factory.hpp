#pragma once

#include <cone_validator/deviation_validator.hpp>
#include <cone_validator/z_score_validator.hpp>
#include <string>

#include "perception/perception_node.hpp"

struct PerceptionParameters;

PerceptionParameters load_adapter_parameters() {
  PerceptionParameters params;

  auto adapter_node = std::make_shared<rclcpp::Node>("perception_adapter");

  double ransac_epsilon = adapter_node->declare_parameter<double>("ransac_epsilon");
  int ransac_n_neighbours = adapter_node->declare_parameter<int>("ransac_n_neighbours");
  int clustering_n_neighbours = adapter_node->declare_parameter<int>("clustering_n_neighbours");
  double clustering_epsilon = adapter_node->declare_parameter<double>("clustering_epsilon");
  double horizontal_resolution = adapter_node->declare_parameter<double>("horizontal_resolution");
  double vertical_resolution = adapter_node->declare_parameter<double>("vertical_resolution");
  std::string ground_removal_algoritm =
      adapter_node->declare_parameter<std::string>("ground_removal");
  std::string target_file = adapter_node->declare_parameter<std::string>("target_file");
  double max_correspondence_distance =
      adapter_node->declare_parameter<double>("max_correspondence_distance");
  int max_iteration = adapter_node->declare_parameter<int>("max_iteration");
  double transformation_epsilon = adapter_node->declare_parameter<double>("transformation_epsilon");
  double euclidean_fitness_epsilon =
      adapter_node->declare_parameter<double>("euclidean_fitness_epsilon");
  double fov_trim = adapter_node->declare_parameter<double>("fov_trim");
  params.adapter_ = adapter_node->declare_parameter<std::string>("adapter");
  params.vehicle_frame_id_ = params.adapter_ == "eufs" ? "velodyne" : "hesai_lidar";
  params.pc_max_range_ = adapter_node->declare_parameter<double>("pc_max_range");

  // Create shared pointers for components
  if (ground_removal_algoritm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_n_neighbours);
  } else if (ground_removal_algoritm == "grid_ransac") {
    int n_angular_grids = adapter_node->declare_parameter<int>("n_angular_grids");
    double radius_resolution = adapter_node->declare_parameter<double>("radius_resolution");
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_n_neighbours,
                                                          n_angular_grids, radius_resolution);
  }

  // Height Validator Parameters
  double min_height = adapter_node->declare_parameter<double>("min_height");
  double max_height = adapter_node->declare_parameter<double>("max_height");
  
  // Deviation Validator Parameters
  double min_xoy = adapter_node->declare_parameter<double>("min_xoy");
  double max_xoy = adapter_node->declare_parameter<double>("max_xoy");
  double min_z = adapter_node->declare_parameter<double>("min_z");
  double max_z = adapter_node->declare_parameter<double>("max_z");
  
  // Z-Score Validator Parameters
  double min_z_score_x = adapter_node->declare_parameter<double>("min_z_score_x");
  double max_z_score_x = adapter_node->declare_parameter<double>("max_z_score_x");
  double min_z_score_y = adapter_node->declare_parameter<double>("min_z_score_y");
  double max_z_score_y = adapter_node->declare_parameter<double>("max_z_score_y");

  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);
  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();

  params.cone_validators_ = {std::make_shared<CylinderValidator>(0.200, 0.325),
                             std::make_shared<HeightValidator>(min_height, max_height),
                             std::make_shared<DeviationValidator>(min_xoy, max_xoy, min_z, max_z),
                             std::make_shared<ZScoreValidator>(min_z_score_x, max_z_score_x,
                                                               min_z_score_y, max_z_score_y)};

  if (params.adapter_ == "eufs") {
    params.cone_validators_ = {};
  }

  params.distance_predict_ =
      std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);

  params.icp_ = std::make_shared<ICP>(target_file, max_correspondence_distance, max_iteration,
                                      transformation_epsilon, euclidean_fitness_epsilon);
  params.fov_trim_ = fov_trim;

  return params;
}
