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
  params.angle_gain_ = adapter_node->declare_parameter("angle_gain", 11.0);
  params.distance_gain_ = adapter_node->declare_parameter("distance_gain", 8.0);
  params.ncones_gain_ = adapter_node->declare_parameter("ncones_gain", 8.7);
  params.angle_exponent_ = adapter_node->declare_parameter("angle_exponent", 5.3);
  params.distance_exponent_ = adapter_node->declare_parameter("distance_exponent", 0.698);
  params.cost_max_ = adapter_node->declare_parameter("cost_max", 40.0);
  params.outliers_spline_order_ =
      static_cast<int>(adapter_node->declare_parameter("outliers_spline_order", 3));
  params.outliers_spline_coeffs_ratio_ =
      static_cast<float>(adapter_node->declare_parameter("outliers_spline_coeffs_ratio", 3.0));
  params.outliers_spline_precision_ =
      static_cast<int>(adapter_node->declare_parameter("outliers_spline_precision", 1));
  params.path_calculation_dist_threshold_ =
      adapter_node->declare_parameter("path_calculation_dist_threshold", 7.0);
  params.smoothing_spline_order_ =
      static_cast<int>(adapter_node->declare_parameter("smoothing_spline_order", 3));
  params.smoothing_spline_coeffs_ratio_ =
      static_cast<float>(adapter_node->declare_parameter("smoothing_spline_coeffs_ratio", 3.0));
  params.smoothing_spline_precision_ =
      static_cast<int>(adapter_node->declare_parameter("smoothing_spline_precision", 10));
  params.publishing_visualization_msgs_ =
      adapter_node->declare_parameter("publishing_visualization_msg", false);
  params.using_simulated_se_ = adapter_node->declare_parameter("use_simulated_se", false);
  params.desired_velocity_ = adapter_node->declare_parameter("pre_defined_velocity_planning", 2);
  std::string adapter_type = adapter_node->declare_parameter("adapter", "vehicle");
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