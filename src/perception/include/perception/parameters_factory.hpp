#pragma once

#include <cone_validator/deviation_validator.hpp>
#include <cone_validator/z_score_validator.hpp>
#include <fov_trimming/cut_trimming.hpp>
#include <string>

#include "perception/perception_node.hpp"

struct PerceptionParameters;

PerceptionParameters load_adapter_parameters() {
  PerceptionParameters params;

  auto adapter_node = std::make_shared<rclcpp::Node>("perception_adapter");
  params.adapter_ = adapter_node->declare_parameter("adapter", "eufs");
  params.vehicle_frame_id_ = params.adapter_ == "eufs" ? "velodyne" : "livox_front";

  // Create shared pointer for Fov Trimming , Fov Trimming Parameters
  double fov_trim_angle = adapter_node->declare_parameter("fov_trim_angle", 90);
  double pc_max_range = adapter_node->declare_parameter("pc_max_range", 15.0);
  double pc_min_range = adapter_node->declare_parameter("pc_min_range", 1.0);
  double pc_rlidar_max_height = adapter_node->declare_parameter("pc_rlidar_max_height", -0.20);
  params.fov_trimming_ = std::make_shared<CutTrimming>(pc_max_range, pc_min_range,
                                                       pc_rlidar_max_height, fov_trim_angle);

  // Create shared pointers for ground removal, ground removal parameters
  std::string ground_removal_algoritm =
      adapter_node->declare_parameter("ground_removal", "grid_ransac");
  double ransac_epsilon = adapter_node->declare_parameter("ransac_epsilon", 0.05);
  int ransac_iterations = adapter_node->declare_parameter("ransac_iterations", 20);
  if (ground_removal_algoritm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_iterations);
  } else if (ground_removal_algoritm == "grid_ransac") {
    int n_angular_grids = adapter_node->declare_parameter("n_angular_grids", 7);
    double radius_resolution = adapter_node->declare_parameter("radius_resolution", 7.5);
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_iterations,
                                                          n_angular_grids, radius_resolution);
  }

  // Create shared pointer for Cultering, Clustering Parameters
  int clustering_n_neighbours = adapter_node->declare_parameter("clustering_n_neighbours", 1);
  double clustering_epsilon = adapter_node->declare_parameter("clustering_epsilon", 0.1);
  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);

  // Height Validator Parameters
  double min_height = adapter_node->declare_parameter("min_height", 0.1);
  double max_height = adapter_node->declare_parameter("max_height", 0.55);

  // Deviation Validator Parameters
  double min_xoy = adapter_node->declare_parameter("min_xoy", 0.0);
  double max_xoy = adapter_node->declare_parameter("max_xoy", 0.3);
  double min_z = adapter_node->declare_parameter("min_z", 0.00001);
  double max_z = adapter_node->declare_parameter("max_z", 0.6);

  // Z-Score Validator Parameters
  double min_z_score_x = adapter_node->declare_parameter("min_z_score_x", 0.45);
  double max_z_score_x = adapter_node->declare_parameter("max_z_score_x", 1.55);
  double min_z_score_y = adapter_node->declare_parameter("min_z_score_y", 0.45);
  double max_z_score_y = adapter_node->declare_parameter("max_z_score_y", 1.55);

  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();

  params.cone_validators_ = {std::make_shared<CylinderValidator>(0.228, 0.325),
                             std::make_shared<HeightValidator>(min_height, max_height),
                             std::make_shared<DeviationValidator>(min_xoy, max_xoy, min_z, max_z),
                             std::make_shared<ZScoreValidator>(min_z_score_x, max_z_score_x,
                                                               min_z_score_y, max_z_score_y)};

  if (params.adapter_ == "eufs") {
    params.cone_validators_ = {};
  }

  // Create shared pointer for Distance prediction, Distance prediction Parameters
  double horizontal_resolution = adapter_node->declare_parameter("horizontal_resolution", 0.33);
  double vertical_resolution = adapter_node->declare_parameter("vertical_resolution", 0.22);
  params.distance_predict_ =
      std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);

  // Create shared pointer for Distance prediction, Distance prediction Parameters
  std::string target_file = adapter_node->declare_parameter("target_file", "cone.pcd");
  double max_correspondence_distance =
      adapter_node->declare_parameter("max_correspondence_distance", 0.1);
  int max_iteration = adapter_node->declare_parameter("max_iteration", 100);
  double transformation_epsilon = adapter_node->declare_parameter("transformation_epsilon", 1e-8);
  double euclidean_fitness_epsilon =
      adapter_node->declare_parameter("euclidean_fitness_epsilon", 1e-6);
  params.icp_ = std::make_shared<ICP>(target_file, max_correspondence_distance, max_iteration,
                                      transformation_epsilon, euclidean_fitness_epsilon);

  return params;
}
