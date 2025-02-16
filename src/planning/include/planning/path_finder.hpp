#pragma once

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/maths/angle_and_norms.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/track_side.hpp"
#include "config/cone_coloring_config.hpp"
#include "rclcpp/rclcpp.hpp"

using Cone = common_lib::structures::Cone;
using Pose = common_lib::structures::Pose;
using Position = common_lib::structures::Position;
using TwoDVector = common_lib::structures::Position;
using TrackSide = common_lib::structures::TrackSide;
using Color = common_lib::competition_logic::Color;
using AngleAndNorms = common_lib::maths::AngleAndNorms;
using PathPoint = common_lib::structures::PathPoint;

class PathFinder {
private:
  ConeColoringConfig config_ = ConeColoringConfig();
  std::vector<PathPoint*> path_ = {};

  /**
   * @brief calculate the expected position of the initial cones
   *
   * @param car_pose car pose in the map relative to the origin
   * @param track_side side of the track of the expected position to be calculated
   */
  Position expected_initial_cone_position(const Pose& car_pose, const TrackSide& track_side) const;

  void find_first_point(std::vector<PathPoint*>& mid_points, const Pose& car_pose);
  double calculate_cost2(const PathPoint* next_cone, const PathPoint* last_cone,
                         const double& colored_to_input_cones_ratio, const Pose& car_pose) const;
  void find_second_point(std::vector<PathPoint*>& candidates, const Pose& car_pose,
                         int& n_colored_cones, const int n_input_cones);

  double calculate_cost(const PathPoint* next_cone, const PathPoint* last_cone,
                        const TwoDVector& previous_to_last_vector,
                        const double& colored_to_input_cones_ratio) const;
  bool try_to_color_next_cone(std::vector<PathPoint*>& uncolored_cones,
                              std::vector<PathPoint*>& colored_cones, int& n_colored_cones,
                              const int n_input_cones);
  std::vector<PathPoint*> find_candidates(std::vector<PathPoint*>& mid_points);

  bool contains(PathPoint* point);

  PathPoint* find_mid_point(std::vector<PathPoint*> mid_points, Cone* cone1, Cone* cone2);

public:
  PathFinder(ConeColoringConfig& config);
  PathFinder() = default;
  std::vector<PathPoint> find_path(std::vector<PathPoint*> cones, const Pose& car_pose);
};