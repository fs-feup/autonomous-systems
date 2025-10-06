#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <cmath>
#include <map>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/mid_point.hpp"
#include "config/path_calculation_config.hpp"
#include "rclcpp/rclcpp.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using DT = CGAL::Delaunay_triangulation_2<K>;
using Point = K::Point_2;

using Vertex_handle = DT::Vertex_handle;
using Finite_edges_iterator = DT::Finite_edges_iterator;

using Cone = common_lib::structures::Cone;
using PathPoint = common_lib::structures::PathPoint;
using MidPoint = common_lib::structures::MidPoint;

/**
 * @brief PathCalculation class for generating local paths.
 *
 * The PathCalculation class contains methods for calculating the best local
 * path and stores input data and results related to path planning.
 */
class PathCalculation {
  /**
   * @brief configuration of the outliers removal algorithm
   *
   */
  PathCalculationConfig config_;

private:
  std::vector<PathPoint> predefined_path_;
  std::vector<Point> past_path_;
  int reset_path_counter_ = 0;
  std::vector<Point> path_to_car;

  // SKIDPAD
  bool skidpad_data_loaded_ = false;
  std::vector<std::pair<double, double>> reference_cones_;
  std::vector<PathPoint> hardcoded_path_;

  
  // Anchor pose for the path, to avoid calculating the path from the position of the car
  common_lib::structures::Pose initial_pose_;
  // The current vehicle pose  
  common_lib::structures::Pose vehicle_pose_;
  bool anchor_pose_set_ = false;
  std::vector<std::shared_ptr<MidPoint>> midpoints_;
  /* Stores the edges of the Delaunay triangulation as pairs of points,
  used for visualization (each pair represents one edge of a triangle).*/
  std::vector<std::pair<Point, Point>> triangulations_;

  // Path construction state members (used during path calculation)
  std::vector<Point> current_path_;
  std::unordered_map<Point, std::shared_ptr<MidPoint>> point_to_midpoint_;
  std::unordered_set<std::shared_ptr<MidPoint>> visited_midpoints_;
  std::unordered_set<std::shared_ptr<Cone>> discarded_cones_;

  /**
   * @brief Recursively finds the best next midpoint in the path using depth-first search
   *
   * @param depth Maximum depth to search
   * @param previous Previous midpoint in the path
   * @param current Current midpoint being evaluated
   * @param maxcost Maximum cost allowed for path segment
   * @return std::pair<double, std::shared_ptr<MidPoint>> Cost and next midpoint pair
   */
  std::pair<double, std::shared_ptr<MidPoint>> find_best_next_midpoint(
      int depth, 
      const std::shared_ptr<MidPoint>& previous, 
      const std::shared_ptr<MidPoint>& current,
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
      const std::shared_ptr<MidPoint>& previous,
      const std::shared_ptr<MidPoint>& current,
      const std::shared_ptr<MidPoint>& next);

  /**
   * @brief Initializes the path using the current pose and the nearest midpoints.
   * 
   * If a precomputed path exists, it snaps to the nearest midpoints and filters out
   * close or already-visited points. Otherwise, it selects the best two starting midpoints
   * based on the vehicle's current pose. Also updates the set of discarded cones along the way.
   */
  void calculate_initial_path();

  /**
   * @brief Snaps points from previous path to valid midpoints
   * 
   */
  void update_path_from_past_path();

  /**
   * @brief Selects initial path from anchor pose when no previous path exists
   */
  void initialize_path_from_anchor();

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
      const std::shared_ptr<MidPoint>& last_mp,
      const std::shared_ptr<MidPoint>& current_mp);

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
  std::shared_ptr<MidPoint> find_nearest_midpoint(const Point& target);
  
  /**
   * @brief Resets the path periodically to avoid long-term drift or degradation
   *
   * If use_sliding_window_ is enabled, updates midpoints_ 
   * to allow the path to be reconstructed again. 
   *
   * @param cone_array   Vector of cones representing the current detected cones.
   * @return int         Updated maximum number of points for the path.
   */
  int reset_path(const std::vector<Cone>& cone_array);

  /**
   * @brief Clears all path construction state variables
   * 
   * Resets current_path_, point_to_midpoint_, visited_midpoints_, and discarded_cones_
   * to prepare for a new path calculation.
   */
  void clear_path_state();

  /**
   * @brief Filters cones to find the ones that will be used for the triangulations.
   *
   * If sliding window mode is enabled, only cones within the configured
   * radius of the vehicle_pose_ are kept. Otherwise all cones are kept.
   *
   * @param cone_array Input list of all cones
   * @param filtered_cones Cones that pass the filter
   */
  void filter_cones(const std::vector<Cone>& cone_array,
                    std::vector<std::shared_ptr<Cone>>& filtered_cones);
  
  /**
   * @brief Creates midpoints between cones based on Delaunay triangulation.
   * 
   * Generates midpoints for cone pairs within configured distance thresholds,
   * avoiding duplicates, and connects neighboring midpoints sharing the same triangle.
   * 
   * @param filtered_cones Input vector of cones with 2D positions.
   */
  void create_midpoints(std::vector<std::shared_ptr<Cone>>& filtered_cones); 

  /**
   * @brief Finds the first and second points to start the path
   *
   * @return std::pair<std::shared_ptr<MidPoint>, std::shared_ptr<MidPoint>> First and second points for the path
   */
  std::pair<std::shared_ptr<MidPoint>, std::shared_ptr<MidPoint>> select_starting_midpoints();

  /**
   * @brief Processes a single edge of a Delaunay triangle
   * 
   * @param va First vertex of the edge
   * @param vb Second vertex of the edge
   * @param filtered_cones Vector of all cones
   * @param segment_to_midpoint Map to track unique midpoints
   * @return std::shared_ptr<MidPoint> Created or existing midpoint, or nullptr if invalid
   */
  std::shared_ptr<MidPoint> process_triangle_edge(
      const Vertex_handle& va,
      const Vertex_handle& vb,
      std::vector<std::shared_ptr<Cone>>& filtered_cones,
      std::map<std::pair<int, int>, std::shared_ptr<MidPoint>>& segment_to_midpoint);

  /**
   * @brief Connects midpoints that share the same triangle
   * 
   * @param triangle_midpoints Array of 3 midpoints from a triangle
   */
  void connect_triangle_midpoints(
      const std::array<std::shared_ptr<MidPoint>, 3>& triangle_midpoints);

  /**
   * @brief Selects candidate midpoints in front of the vehicle
   * 
   * @param anchor_pose Reference pose to select from
   * @param num_candidates Number of candidates to select
   * @return std::vector<std::shared_ptr<MidPoint>> Selected candidate midpoints
   */
  std::vector<std::shared_ptr<MidPoint>> select_candidate_midpoints(
      const MidPoint& anchor_pose,
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
  explicit PathCalculation(const PathCalculationConfig& config) : config_(config) {}


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
  std::vector<PathPoint> get_global_path() const;

  const std::vector<std::pair<Point, Point>>& get_triangulations() const;
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_