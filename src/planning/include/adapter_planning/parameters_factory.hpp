#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "adapter_planning/eufs.hpp"
#include "adapter_planning/fsds.hpp"
#include "adapter_planning/pacsim.hpp"
#include "adapter_planning/vehicle.hpp"
#include "planning/planning.hpp"

struct PlanningParameters;

std::string load_adapter_parameters(PlanningParameters& params) {
  auto adapter_node = std::make_shared<rclcpp::Node>("planning_adapter");
  params.angle_gain_ = adapter_node->declare_parameter<double>("angle_gain");
  params.distance_gain_ = adapter_node->declare_parameter<double>("distance_gain");
  params.ncones_gain_ = adapter_node->declare_parameter<double>("ncones_gain");
  params.angle_exponent_ = adapter_node->declare_parameter<double>("angle_exponent");
  params.distance_exponent_ = adapter_node->declare_parameter<double>("distance_exponent");
  params.same_cone_distance_threshold_ =
      adapter_node->declare_parameter<double>("same_cone_distance_threshold");
  params.cost_max_ = adapter_node->declare_parameter<double>("cost_max");
  params.use_memory_cone_coloring_ =
      adapter_node->declare_parameter<bool>("use_memory_cone_coloring");
  params.outliers_spline_order_ =
      static_cast<int>(adapter_node->declare_parameter<int>("outliers_spline_order"));
  params.outliers_spline_coeffs_ratio_ =
      static_cast<float>(adapter_node->declare_parameter<double>("outliers_spline_coeffs_ratio"));
  params.outliers_spline_precision_ =
      static_cast<int>(adapter_node->declare_parameter<int>("outliers_spline_precision"));
  params.path_calculation_dist_threshold_ =
      adapter_node->declare_parameter<double>("path_calculation_dist_threshold");
  params.smoothing_spline_order_ =
      static_cast<int>(adapter_node->declare_parameter<int>("smoothing_spline_order"));
  params.smoothing_spline_coeffs_ratio_ =
      static_cast<float>(adapter_node->declare_parameter<double>("smoothing_spline_coeffs_ratio"));
  params.smoothing_spline_precision_ =
      static_cast<int>(adapter_node->declare_parameter<int>("smoothing_spline_precision"));
  params.publishing_visualization_msgs_ =
      adapter_node->declare_parameter<bool>("publishing_visualization_msg");
  params.using_simulated_se_ = adapter_node->declare_parameter<bool>("use_simulated_se");
  params.desired_velocity_ =
      static_cast<double>(adapter_node->declare_parameter<double>("pre_defined_velocity_planning"));
  params.use_outlier_removal_ = adapter_node->declare_parameter<bool>("use_outlier_removal");
  params.use_path_smoothing_ = adapter_node->declare_parameter<bool>("use_path_smoothing");
  std::string adapter_type = adapter_node->declare_parameter<std::string>("adapter");
  params.map_frame_id_ = adapter_type == "eufs" ? "base_footprint" : "map";

  return adapter_type;
}

std::shared_ptr<Planning> create_planning(const std::string_view& adapter_type,
                                          const PlanningParameters& params) {
  static const std::unordered_map<
      std::string_view, std::function<std::shared_ptr<Planning>(const PlanningParameters&)>>
      adapter_map = {
          {"vehicle",
           [](const PlanningParameters& parameters) {
             return std::make_shared<VehicleAdapter>(parameters);
           }},
          {"pacsim",
           [](const PlanningParameters& parameters) {
             return std::make_shared<PacSimAdapter>(parameters);
           }},
          {"eufs",
           [](const PlanningParameters& parameters) {
             return std::make_shared<EufsAdapter>(parameters);
           }},
          {"fsds",
           [](const PlanningParameters& parameters) {
             return std::make_shared<FsdsAdapter>(parameters);
           }},
      };

  auto it = adapter_map.find(adapter_type);
  if (it != adapter_map.end()) {
    return it->second(params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("planning"), "Adapter type not recognized");
    return nullptr;
  }
}