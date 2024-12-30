#pragma once

#include <cone_validator/deviation_validator.hpp>
#include <cone_validator/displacement_validator.hpp>
#include <cone_validator/npoints_validator.hpp>
#include <cone_validator/z_score_validator.hpp>
#include <fov_trimming/cut_trimming.hpp>
#include <string>

#include "perception/perception_node.hpp"

struct PerceptionParameters;

PerceptionParameters load_adapter_parameters() {
  PerceptionParameters params;

  auto adapter_node = std::make_shared<rclcpp::Node>("perception_adapter");
  params.adapter_ = adapter_node->declare_parameter("adapter", "eufs");
  params.vehicle_frame_id_ = adapter_node->declare_parameter("vehicle_frame_id", "hesai_lidar");

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

  // Create shared pointer for Clustering, Clustering Parameters
  int clustering_n_neighbours = adapter_node->declare_parameter("clustering_n_neighbours", 1);
  double clustering_epsilon = adapter_node->declare_parameter("clustering_epsilon", 0.1);
  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);

  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();

  // Number of points Validator Parameters
  long unsigned int min_n_points = adapter_node->declare_parameter("min_n_points", 4);

  // Height Validator Parameters
  double min_height = adapter_node->declare_parameter("min_height", 0.1);
  double large_max_height = adapter_node->declare_parameter("large_max_height", 0.55);
  double small_max_height = adapter_node->declare_parameter("small_max_height", 0.4);
  double height_cap = adapter_node->declare_parameter("height_cap", 0.5);

  // Deviation Validator Parameters
  double min_xoy = adapter_node->declare_parameter("min_xoy", 0.0);
  double max_xoy = adapter_node->declare_parameter("max_xoy", 0.3);
  double min_z = adapter_node->declare_parameter("min_z", 0.00001);
  double max_z = adapter_node->declare_parameter("max_z", 0.6);

  // Displacement Validator Parameters
  double min_distance_x = adapter_node->declare_parameter("min_distance_x", 0.1);
  double min_distance_y = adapter_node->declare_parameter("min_distance_y", 0.1);
  double min_distance_z = adapter_node->declare_parameter("min_distance_z", 0.25);

  // Z-Score Validator Parameters
  double min_z_score_x = adapter_node->declare_parameter("min_z_score_x", 0.45);
  double max_z_score_x = adapter_node->declare_parameter("max_z_score_x", 1.55);
  double min_z_score_y = adapter_node->declare_parameter("min_z_score_y", 0.45);
  double max_z_score_y = adapter_node->declare_parameter("max_z_score_y", 1.55);

  // Cylinder Validator Parameters
  double out_distance_cap = adapter_node->declare_parameter("out_distance_cap", 0.5);

  auto cone_validators =
      std::make_shared<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>(
          std::unordered_map<std::string, std::shared_ptr<ConeValidator>>{
              {"npoints", std::make_shared<NPointsValidator>(min_n_points)},
              {"height", std::make_shared<HeightValidator>(min_height, large_max_height,
                                                           small_max_height, height_cap)},
              {"cylinder",
               std::make_shared<CylinderValidator>(0.228, 0.325, 0.285, 0.505, out_distance_cap)},
              {"deviation", std::make_shared<DeviationValidator>(min_xoy, max_xoy, min_z, max_z)},
              {"displacement", std::make_shared<DisplacementValidator>(
                                   min_distance_x, min_distance_y, min_distance_z)},
              {"zscore", std::make_shared<ZScoreValidator>(min_z_score_x, max_z_score_x,
                                                           min_z_score_y, max_z_score_y)}});

  // Weight values for cone evaluator

  // Height weights
  double height_out_weight = adapter_node->declare_parameter("height_out_weight", 0.15);
  double height_in_weight = adapter_node->declare_parameter("height_in_weight", 0.15);

  // Cylinder weights
  double cylinder_radius_weight = adapter_node->declare_parameter("cylinder_radius_weight", 0.15);
  double cylinder_height_weight = adapter_node->declare_parameter("cylinder_height_weight", 0.15);
  double cylinder_npoints_weight = adapter_node->declare_parameter("cylinder_npoints_weight", 0.05);
  double npoints_weight = adapter_node->declare_parameter("npoints_weight", 0.1);

  // Displacement weights
  double displacement_x_weight = adapter_node->declare_parameter("displacement_x_weight", 0.05);
  double displacement_y_weight = adapter_node->declare_parameter("displacement_y_weight", 0.05);
  double displacement_z_weight = adapter_node->declare_parameter("displacement_z_weight", 0.05);

  // Deviation weights
  double deviation_xoy_weight = adapter_node->declare_parameter("deviation_xoy_weight", 0.2);
  double deviation_z_weight = adapter_node->declare_parameter("deviation_z_weight", 0.1);

  // Weights maps shared pointer setup
  auto weight_values = std::make_shared<std::unordered_map<std::string, double>>(
      std::unordered_map<std::string, double>{{"height_out_weight", height_out_weight},
                                              {"height_in_weight", height_in_weight},
                                              {"cylinder_radius_weight", cylinder_radius_weight},
                                              {"cylinder_height_weight", cylinder_height_weight},
                                              {"cylinder_npoints_weight", cylinder_npoints_weight},
                                              {"npoints_weight", npoints_weight},
                                              {"displacement_x_weight", displacement_x_weight},
                                              {"displacement_y_weight", displacement_y_weight},
                                              {"displacement_z_weight", displacement_z_weight},
                                              {"deviation_xoy_weight", deviation_xoy_weight},
                                              {"deviation_z_weight", deviation_z_weight}});
  // Minimum confidence
  double min_confidence = adapter_node->declare_parameter("min_confidence", 1.0);

  params.cone_evaluator_ =
      std::make_shared<ConeEvaluator>(cone_validators, weight_values, min_confidence);

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
