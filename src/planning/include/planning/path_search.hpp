#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_SEARCH_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_SEARCH_HPP_

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/maths/angle_and_norms.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/track_side.hpp"
#include "config/path_search_config.hpp"
#include "rclcpp/rclcpp.hpp"

using PathPoint = common_lib::structures::PathPoint;
using Position = common_lib::structures::Position;
using TwoDVector = common_lib::structures::Position;
using Color = common_lib::competition_logic::Color;
using AngleAndNorms = common_lib::maths::AngleAndNorms;
using Pose = common_lib::structures::Pose;

class PathSearch {
  PathSearchConfig config_;

  double calculate_cost(const PathPoint& next_cone, const PathPoint& last_cone,
                        const TwoDVector& previous_to_last_vector,
                        const double& colored_to_input_cones_ratio) const;
  bool try_to_color_next_cone(
      std::unordered_set<PathPoint, std::hash<PathPoint>>& remaining_pathpoints,
      std::vector<PathPoint>& current_path, int& n_current_pathpoints,
      const int n_input_cones) const;

  PathPoint find_initial_cone(const std::unordered_set<PathPoint, std::hash<PathPoint>>& cones,
                              const Pose& car_pose) const;

  void place_second_cones(std::unordered_set<PathPoint, std::hash<PathPoint>>& input_pathpoints,
                          std::vector<PathPoint>& final_path, const Pose& car_pose,
                          int& n_colored_cones) const;

  void place_initial_cones(std::unordered_set<PathPoint, std::hash<PathPoint>>& input_pathpoints,
                           std::vector<PathPoint>& final_path, const Pose& car_pose,
                           int& n_current_pathpoints) const;

public:
  PathSearch() = default;
  PathSearch(const PathSearchConfig& config) : config_(config) {}

  std::vector<PathPoint> find_path(const std::vector<PathPoint>& path_points,
                                   const Pose& car_pose) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_PATH_SEARCH_HPP_