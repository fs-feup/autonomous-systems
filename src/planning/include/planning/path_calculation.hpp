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
using Color = common_lib::competition_logic::Color;

/**
 * @class PathCalculation
 * @brief Generates optimal local paths.
 * 
 * PathCalculation computes navigation paths by creating midpoints between
 * track cones and searching for optimal sequences using depth-first search
 * with cost-based evaluation. It maintains continuity with previous paths
 * and discards cones as they are passed.
 */
class PathCalculation {
public:
  /**
   * @brief Construct a new default PathCalculation object.
   */
  PathCalculation() = default;

  /**
   * @brief Construct a new PathCalculation object with configuration.
   * @param config Configuration parameters for path calculation behavior.
   */
  explicit PathCalculation(const PathCalculationConfig& config)
      : config_(config), midpoint_generator_(config.midpoint_generator_) {}

  // ===================== Public Core Methods =====================

  /**
   * @brief Generate a path from an array of cones.
   *
   * Processes cone positions to create an optimal navigation path. Updates
   * the internal state to maintain path continuity between invocations.
   *
   * @param cone_array Array of cones representing the track.
   * @return Vector of path points representing the calculated path.
   */
  std::vector<PathPoint> calculate_path(const std::vector<Cone>& cone_array);

  /**
   * @brief Generate a closed loop path for trackdrive competition.
   *
   * Creates a path that forms a complete loop around the track, with
   * interpolated connections and overlap points for continuous traversal.
   *
   * @param cone_array Array of cones representing the track.
   * @return Vector of path points forming a closed loop.
   */
  std::vector<PathPoint> calculate_trackdrive(const std::vector<Cone>& cone_array);

  /**
   * @brief Set the current vehicle position and orientation.
   *
   * Propagates the vehicle pose to the midpoint generator and maintains
   * the current pose for path calculations.
   *
   * @param vehicle_pose The current vehicle position and orientation.
   */
  void set_vehicle_pose(const common_lib::structures::Pose& vehicle_pose);

  // ===================== Public Accessor Methods =====================

  /**
   * @brief Get the path from the start to a lookback distance behind the car’s current position.
   * @return Vector of path points representing the global path.
   */
  std::vector<PathPoint> get_path_to_car() const;

  /**
   * @brief Get the triangulation segments used in path generation.
   * @return Reference to vector of point pairs representing triangulation edges.
   */
  const std::vector<std::pair<Point, Point>>& get_triangulations() const;

  std::vector<Cone> get_cones() const; 

private:
  // ===================== Configuration and State =====================

  PathCalculationConfig config_;
  MidpointGenerator midpoint_generator_;

  // Vehicle and path state
  common_lib::structures::Pose initial_pose_;
  common_lib::structures::Pose vehicle_pose_;
  bool initial_pose_set_ = false;

  // Path storage
  std::vector<Point> path_to_car_;
  std::vector<Point> past_path_;
  int reset_path_counter_ = 0;
  std::vector<std::shared_ptr<Cone>> past_cones_;

  // Midpoints for the current track
  std::vector<std::shared_ptr<Midpoint>> midpoints_;

  // Path construction state (temporary during calculation)
  std::vector<Point> current_path_;
  std::unordered_map<Point, std::shared_ptr<Midpoint>> point_to_midpoint_;
  std::unordered_set<std::shared_ptr<Midpoint>> visited_midpoints_;
  std::unordered_set<std::shared_ptr<Cone>> discarded_cones_;
  std::vector<std::shared_ptr<Cone>> current_cones_;

  // ===================== Path Calculation Methods =====================

  /**
   * @brief Initialize the path from the vehicle's initial pose.
   *
   * Selects the first two midpoints ahead of the vehicle to start the path
   * when no previous path exists.
   */
  void initialize_path_from_initial_pose();

  /**
   * @brief Initialize the path from the previous path.
   *
   * Snaps past path points to valid midpoints and carries forward as much
   * of the previous path as possible, respecting distance constraints.
   */
  void update_path_from_past_path();

  /**
   * @brief Extend the path by adding midpoints.
   *
   * Iteratively searches for optimal next midpoints using depth-first search
   * until reaching the maximum point limit or exhausting valid options.
   *
   * @param max_points Maximum number of new points to add.
   */
  void extend_path(int max_points);

  /**
   * @brief Reset path construction state.
   *
   * Clears all internal tracking structures (current path, visited midpoints,
   * discarded cones) to prepare for a fresh path calculation.
   */
  void clear_path_state();

  /**
   * @brief Handle path reset when completing a lap.
   *
   * Adjusts the maximum points allowed and clears historical path data
   * when completing a circuit.
   *
   * @param should_reset Whether to trigger a full path reset.
   * @return Adjusted maximum points for path extension.
   */
  int reset_path(bool should_reset);

  // ===================== Path Search and Optimization =====================

  /**
   * @brief Select the first two midpoints to start the path.
   *
   * Identifies candidate midpoints ahead of the vehicle and performs a
   * cost-based search to select the best starting pair.
   *
   * @return Pair of midpoint pointers for path initialization.
   */
  std::pair<std::shared_ptr<Midpoint>, std::shared_ptr<Midpoint>> select_starting_midpoints();

  /**
   * @brief Select candidate midpoints in front of the vehicle.
   *
   * Uses a priority queue to identify the most promising midpoints ahead
   * of the vehicle based on combined distance and angle cost.
   *
   * @param anchor_pose Reference pose to evaluate candidates from.
   * @param num_candidates Number of candidates to return.
   * @return Vector of candidate midpoint pointers.
   */
  std::vector<std::shared_ptr<Midpoint>> select_candidate_midpoints(
      const Midpoint& anchor_pose, int num_candidates) const;

  /**
   * @brief Recursively find the best next midpoint using depth-first search.
   *
   * Performs a bounded DFS to evaluate continuation paths from the current
   * midpoint, balancing between direction consistency and distance.
   *
   * @param depth Current search depth (decrements toward base case).
   * @param previous Previous midpoint in the path.
   * @param current Current midpoint being evaluated.
   * @param max_cost Cost threshold for pruning branches.
   * @return Pair of accumulated cost and best next midpoint pointer.
   */
  std::pair<double, std::shared_ptr<Midpoint>> find_best_next_midpoint(
      int depth, const std::shared_ptr<Midpoint>& previous,
      const std::shared_ptr<Midpoint>& current, double max_cost) const;

  /**
   * @brief Calculate the cost of transitioning through three consecutive midpoints.
   *
   * Combines angle deviation (penalizing sharp turns) and distance (penalizing
   * inefficient routing) with configurable exponents and gains.
   *
   * @param previous The midpoint before current.
   * @param current The current midpoint.
   * @param next The candidate next midpoint.
   * @return Combined cost of the transition.
   */
  double calculate_midpoint_cost(const std::shared_ptr<Midpoint>& previous,
                                  const std::shared_ptr<Midpoint>& current,
                                  const std::shared_ptr<Midpoint>& next) const;

  // ===================== Cone Discarding Methods =====================

  /**
   * @brief Invalidate cones and midpoints along the last path segment.
   *
   * Marks cones as discarded if they were on the side of the path that
   * has been passed. Invalidates any midpoints that depend on discarded
   * cones and removes invalid neighbors from connectivity lists.
   */
  void discard_cones_along_path();

  /**
   * @brief Determine which cone to discard between two consecutive midpoints.
   *
   * Identifies the cone from the previous midpoint that is not used by
   * the current midpoint, indicating it has been passed.
   *
   * @param last_mp The previous midpoint in the path.
   * @param current_mp The current midpoint in the path.
   */
  void discard_cone(const std::shared_ptr<Midpoint>& last_mp,
                    const std::shared_ptr<Midpoint>& current_mp);

  /**
   * @brief Mark midpoints as invalid if they use discarded cones.
   *
   * Iterates through all midpoints and invalidates those whose defining
   * cones have been marked as discarded.
   */
  void invalidate_midpoints_with_discarded_cones();

  /**
   * @brief Remove invalid neighbors from all midpoint connection lists.
   *
   * Filters out neighbors that have been invalidated from each midpoint's
   * close_points list to maintain consistency.
   */
  void remove_invalid_neighbors();

  // ===================== Utility Methods =====================

  /**
   * @brief Find the nearest valid midpoint to a target position.
   *
   * Performs a linear search through all midpoints to locate the closest
   * one within the tolerance distance.
   *
   * @param target The target point to search around.
   * @return Pointer to the nearest midpoint, or nullptr if none within tolerance.
   */
  std::shared_ptr<Midpoint> find_nearest_midpoint(const Point& target) const;

  /**
   * @brief Find the optimal point to close a track loop.
   *
   * Evaluates each point in the path to determine which produces the
   * smoothest closure back to the starting point.
   *
   * @param path The path points to analyze.
   * @return Index of the best cutoff point.
   */
  int find_best_loop_closure(const std::vector<PathPoint>& path) const;

  /**
   * @brief Generate interpolated points between two path endpoints.
   *
   * Creates evenly-spaced intermediate points for smooth trajectory between
   * a start and end point.
   *
   * @param start Starting path point.
   * @param end Ending path point.
   * @param num_points Number of intermediate points to generate.
   * @return Vector of interpolated path points.
   */
  std::vector<PathPoint> add_interpolated_points(const PathPoint& start, const PathPoint& end,
                                                  int num_points) const;

  /**
 * @brief Converts a vector of Point objects into a vector of PathPoint objects.
 *
 * @param points A vector containing the input Point objects.
 * @return A vector of PathPoint objects constructed from the given points.
 */
  std::vector<PathPoint> get_path_points_from_points(const std::vector<Point>& points) const;

  void color_first_cones(std::shared_ptr<Midpoint> first_point);

  bool color_cone(std::shared_ptr<Midpoint> point);

  

};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_