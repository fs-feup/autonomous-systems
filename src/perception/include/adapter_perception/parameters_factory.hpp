#pragma once

#include <string>
#include <vector>

#include "adapter_perception/eufs.hpp"
#include "adapter_perception/fsds.hpp"
#include "adapter_perception/vehicle.hpp"
#include "perception/perception_node.hpp"

struct PerceptionParameters;

std::string load_adapter_parameters(PerceptionParameters& params) {
  auto adapter_node = std::make_shared<rclcpp::Node>("perception_adapter");

  double ransac_epsilon = adapter_node->declare_parameter("ransac_epsilon", 0.1);
  int ransac_n_neighbours = adapter_node->declare_parameter("ransac_n_neighbours", 30);
  int clustering_n_neighbours = adapter_node->declare_parameter("clustering_n_neighbours", 3);
  double clustering_epsilon = adapter_node->declare_parameter("clustering_epsilon", 0.1);
  double horizontal_resolution = adapter_node->declare_parameter("horizontal_resolution", 0.33);
  double vertical_resolution = adapter_node->declare_parameter("vertical_resolution", 0.22);
  std::string ground_removal_algoritm = adapter_node->declare_parameter("ground_removal", "ransac");

  // Create shared pointers for components
  if (ground_removal_algoritm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_n_neighbours);
  } else if (ground_removal_algoritm == "grid_ransac") {
    int n_angular_grids = adapter_node->declare_parameter("n_angular_grids", 6);
    double radius_resolution = adapter_node->declare_parameter("radius_resolution", 10.0);
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_n_neighbours,
                                                          n_angular_grids, radius_resolution);
  }

  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);
  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();
  params.cone_validators_ = {std::make_shared<CylinderValidator>(0.228, 0.325),
                             std::make_shared<HeightValidator>(0.325)};
  params.distance_predict_ =
      std::make_shared<DistancePredict>(vertical_resolution, horizontal_resolution);

  return adapter_node->declare_parameter("adapter", "vehicle");
}

std::shared_ptr<Perception> create_perception(const std::string_view& adapter_type,
                                              const PerceptionParameters& params) {
  static const std::unordered_map<
      std::string_view, std::function<std::shared_ptr<Perception>(const PerceptionParameters&)>>
      adapter_map = {
          {"vehicle",
           [](const PerceptionParameters& parameters) {
             return std::make_shared<VehicleAdapter>(parameters);
           }},
          {"eufs",
           [](const PerceptionParameters& parameters) {
             return std::make_shared<EufsAdapter>(parameters);
           }},
          {"fsds",
           [](const PerceptionParameters& parameters) {
             return std::make_shared<FsdsAdapter>(parameters);
           }},
      };

  auto it = adapter_map.find(adapter_type);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("perception"), "Adapter type not recognized");
    return nullptr;
  }
}