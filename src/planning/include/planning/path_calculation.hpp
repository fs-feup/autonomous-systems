#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_

#include <cmath>
#include <map>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/midpoint.hpp"
#include "config/path_calculation_config.hpp"
#include "planning/midpoint_generator.hpp"
#include "rclcpp/rclcpp.hpp"

using Cone = common_lib::structures::Cone;
using PathPoint = common_lib::structures::PathPoint;
using Midpoint = common_lib::structures::Midpoint;

/**
 * @brief PathCalculation class for generating local paths.
 *
 * The PathCalculation class contains methods for calculating the best local
 * path and stores input data and results related to path planning.
 */
class PathCalculation {

private:
  /**
   * @brief configuration of the outliers removal algorithm
   *
   */
  PathCalculationConfig config_;
  MidpointGenerator midpoint_generator_;

  std::vector<PathPoint> predefined_path_;
  std::vector<Point> past_path_;
  int reset_path_counter_ = 0;
  std::vector<Point> path_to_car;
  
  // Anchor pose for the path, to avoid calculating the path from the position of the car
  common_lib::structures::Pose initial_pose_;
  // The current vehicle pose  
  common_lib::structures::Pose vehicle_pose_;
  bool initial_pose_set_ = false;
  std::vector<std::shared_ptr<Midpoint>> midpoints_;

  // Path construction state members (used during path calculation)
  std::vector<Point> current_path_;
  std::unordered_map<Point, std::shared_ptr<Midpoint>> point_to_midpoint_;
  std::unordered_set<std::shared_ptr<Midpoint>> visited_midpoints_;
  std::unordered_set<std::shared_ptr<Cone>> discarded_cones_;

  /**
   * @brief Recursively finds the best next midpoint in the path using depth-first search
   *
   * @param depth Maximum depth to search
   * @param previous Previous midpoint in the path
   * @param current Current midpoint being evaluated
   * @param maxcost Maximum cost allowed for path segment
   * @return std::pair<double, std::shared_ptr<Midpoint>> Cost and next midpoint pair
   */
  std::pair<double, std::shared_ptr<Midpoint>> find_best_next_midpoint(
      int depth, 
      const std::shared_ptr<Midpoint>& previous, 
      const std::shared_ptr<Midpoint>& current,
      double maxcost);

  /**
   * @brief Calculates the cost between two midpoints
   * 
   * @param previous Previous midpoint
   * @param current Current midpoint
   * @param next Next midpoint candidate
   * @return double The calculated cost
   */
  double calculate_midpoint_cost(
      const std::shared_ptr<Midpoint>& previous,
      const std::shared_ptr<Midpoint>& current,
      const std::shared_ptr<Midpoint>& next);

  /**
   * @brief Snaps points from previous path to valid midpoints
   * 
   */
  void update_path_from_past_path();

  /**
   * @brief Selects initial path from anchor pose when no previous path exists
   */
  void initialize_path_from_initial_pose();

  /**
   * @brief Extends the current path by exploring nearby midpoints.
   * 
   * Iteratively adds new midpoints to the path using a DFS-based cost search,
   * respecting a maximum number of points and cost constraints. 
   * Updates the set of visited midpoints and discards cones along the extended path.
   * 
   * @param max_points Maximum number of new points to add to the path.
   */
  void extend_path(int max_points);

  /**
   * @brief Invalidates cones and midpoints based on the last path segment.
   * 
   * Identifies which cone between the last two midpoints should be discarded,
   * marks associated midpoints as invalid, and removes invalid neighbors from their connections.
   */
  void discard_cones_along_path();

  /**
   * @brief Identifies the cone to discard between two consecutive midpoints
   * 
   * @param last_mp Previous midpoint
   * @param current_mp Current midpoint
   */
  void discard_cone(
      const std::shared_ptr<Midpoint>& last_mp,
      const std::shared_ptr<Midpoint>& current_mp);

  /**
   * @brief Marks midpoints as invalid if they use discarded cones
   */
  void invalidate_midpoints_with_discarded_cones();

  /**
   * @brief Removes invalid neighbors from all midpoints
   */
  void remove_invalid_neighbors();

  /**
   * @brief Find the nearest midpoint to a target point within a tolerance
   * 
   * @param target The target point to find the nearest midpoint for
   * @return std::shared_ptr<MidPoint> Pointer to the nearest midpoint, or nullptr if none found
   */
  std::shared_ptr<Midpoint> find_nearest_midpoint(const Point& target);
  

  int reset_path(bool reset);

  /**
   * @brief Clears all path construction state variables
   * 
   * Resets current_path_, point_to_midpoint_, visited_midpoints_, and discarded_cones_
   * to prepare for a new path calculation.
   */
  void clear_path_state();

  /**
   * @brief Finds the first and second points to start the path
   *
   * @return std::pair<std::shared_ptr<MidPoint>, std::shared_ptr<MidPoint>> First and second points for the path
   */
  std::pair<std::shared_ptr<Midpoint>, std::shared_ptr<Midpoint>> select_starting_midpoints();

  /**
   * @brief Selects candidate midpoints in front of the vehicle
   * 
   * @param anchor_pose Reference pose to select from
   * @param num_candidates Number of candidates to select
   * @return std::vector<std::shared_ptr<MidPoint>> Selected candidate midpoints
   */
  std::vector<std::shared_ptr<Midpoint>> select_candidate_midpoints(
      const Midpoint& anchor_pose,
      int num_candidates);

  /**
   * @brief Finds the best point to close the track loop
   * 
   * @param path The path to analyze
   * @return int Index of the best cutoff point
   */
  int find_best_loop_closure(const std::vector<PathPoint>& path);

  /**
   * @brief Adds interpolated points between two path points
   * 
   * @param start Starting point
   * @param end Ending point
   * @param num_points Number of intermediate points to add
   * @return std::vector<PathPoint> Interpolated points
   */
  std::vector<PathPoint> add_interpolated_points(
      const PathPoint& start,
      const PathPoint& end,
      int num_points);

public:
  /**
   * @brief Construct a new default PathCalculation object
   *
   */
  PathCalculation() = default;

  /**
   * @brief Constructor for PathCalculation with a given configuration.
   *
   * @param config Config object with PathCalculation configs.
   */
  explicit PathCalculation(const PathCalculationConfig& config) 
    : config_(config),
    midpoint_generator_(config.midpoint_generator_) {}


  /**
   * @brief Generate a path from cone array without color information
   *
   * @param cone_array The array of cones representing the track
   * @return std::vector<PathPoint> The generated path
   */
  std::vector<PathPoint> calculate_path(std::vector<Cone>& cone_array);

  /**
   * @brief Updates the vehicle pose
   *
   * @param vehicle_pose The current vehicle pose
   */
  void update_vehicle_pose(const common_lib::structures::Pose& vehicle_pose);
  /**
   * @brief Generate a path for trackdrive course
   * @returns a vector of PathPoint objects representing the path.
   */
  std::vector<PathPoint> calculate_trackdrive(std::vector<Cone>& cone_array);

  /**
   * @brief Get the global path
   * 
   * @return std::vector<PathPoint> The global path
   */                                  
  std::vector<PathPoint> get_path_to_car() const;

  const std::vector<std::pair<Point, Point>>& get_triangulations() const;

  
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_