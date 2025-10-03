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
  bool path_orientation_corrected_ = false;
  std::vector<PathPoint> predefined_path_;
  std::vector<Point> global_path_;
  int path_update_counter_ = 0;
  std::vector<Point> path_to_car;

  // SKIDPAD
  bool skidpad_data_loaded_ = false;
  std::vector<std::pair<double, double>> reference_cones_;
  std::vector<PathPoint> hardcoded_path_;

  
  // Anchor pose for the path, to avoid calculating the path from the position of the car
  common_lib::structures::Pose anchor_pose_;
  // The current vehicle pose  
  common_lib::structures::Pose vehicle_pose_;
  bool anchor_pose_set_ = false;
  std::vector<std::shared_ptr<MidPoint>> mid_points_;


public:

  std::vector<std::pair<Point, Point>> triangulations;

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
   * @brief Depth-first search for path cost calculation
   *
   * @param depth Maximum depth to search
   * @param previous Previous point in the path
   * @param current Current point being evaluated
   * @param maxcost Maximum cost allowed for path segment
   * @return std::pair<double, MidPoint*> Cost and next point pair
   */
  std::pair<double, MidPoint*> dfs_cost(int depth, const MidPoint* previous, MidPoint* current,
                                        double maxcost);
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
   * @brief Generate a path from cone array without color information
   *
   * @param cone_array The array of cones representing the track
   * @return std::vector<PathPoint> The generated path
   */
  std::vector<PathPoint> no_coloring_planning(std::vector<Cone>& cone_array);

  /**
   * @brief Updates the anchor point if not already set
   *
   * @param pose The current vehicle pose
   */
  void update_vehicle_pose(const common_lib::structures::Pose& vehicle_pose);

  /**
   * @brief Updates the anchor point if not already set
   *
   * @param pose The current vehicle pose
   */
  void update_anchor_pose();

  /**
   * @brief Finds the first and second points to start the path
   *
   * @return std::pair<MidPoint*, MidPoint*> First and second points for the path
   */
  std::pair<MidPoint*, MidPoint*> find_path_start_points();

  /**
   * @brief Generate a path using DFS cost search
   *
   * @param first The first point of the path
   * @param second The second point of the path
   * @return std::vector<MidPoint*> The generated path as midpoints
   */
  std::vector<MidPoint*> generatePath(MidPoint* first, MidPoint* second);

  /**
   * @brief Convert midpoint path to path points
   *
   * @param path Vector of midpoints representing the path
   * @return std::vector<PathPoint> The final path points
   */
  std::vector<PathPoint> convertToPathPoints(const std::vector<MidPoint*>& path);

  /**
   * @brief Generate a path for skidpad course
   *
   * @param cone_array The array of cones representing the track
   * @param pose The current pose of the vehicle
   * @return std::vector<PathPoint> The generated path
   */
  std::vector<PathPoint> skidpad_path(const std::vector<Cone>& cone_array,
                                      common_lib::structures::Pose pose);

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


  /**
   * @brief Creates midpoints between cones based on Delaunay triangulation.
   * 
   * Generates midpoints for cone pairs within configured distance thresholds,
   * avoiding duplicates, and connects neighboring midpoints sharing the same triangle.
   * 
   * @param filtered_cones Input vector of cones with 2D positions.
   */
  void create_mid_points(std::vector<std::shared_ptr<Cone>>& filtered_cones); 



  /**
   * @brief Initializes the path using the current pose and the nearest midpoints.
   * 
   * If a precomputed path exists, it snaps to the nearest midpoints and filters out
   * close or already-visited points. Otherwise, it selects the best two starting midpoints
   * based on the vehicle's current pose. Also updates the set of discarded cones along the way.
   * 
   * @param path Output vector to store the selected path points.
   * @param point_to_midpoint Map for fast lookup from CGAL point to midpoint.
   * @param visited_midpoints Set of midpoints already used in the path.
   * @param discarded_cones Set of cones to discard along the generated path.
   */
  void calculate_initial_path(
      std::vector<Point>& path,
      const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
      std::unordered_set<MidPoint*>& visited_midpoints,
      std::unordered_set<std::shared_ptr<Cone>>& discarded_cones
  );

  /**
   * @brief Extends the current path by exploring nearby midpoints.
   * 
   * Iteratively adds new midpoints to the path using a DFS-based cost search,
   * respecting a maximum number of points and cost constraints. 
   * Updates the set of visited midpoints and discards cones along the extended path.
   * 
   * @param path Output path to be extended.
   * @param point_to_midpoint Map for fast lookup from CGAL point to midpoint.
   * @param visited_midpoints Set of midpoints already used in the path.
   * @param discarded_cones Set of cones to discard along the extended path.
   * @param max_points Maximum number of new points to add to the path.
   */
  void extend_path(
    std::vector<Point>& path,
    const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints,
    std::unordered_set<std::shared_ptr<Cone>>& discarded_cones,
    int max_points
  );

  /**
   * @brief Invalidates cones and midpoints based on the last path segment.
   * 
   * Identifies which cone between the last two midpoints should be discarded,
   * marks associated midpoints as invalid, and removes invalid neighbors from their connections.
   * 
   * @param path Current path used to determine which cone to discard.
   * @param point_to_midpoint Map for quick access from CGAL point to midpoint.
   * @param discarded_cones Set to store cones marked as discarded.
   */
  void discard_cones_along_path(
    const std::vector<Point>& path,
    const std::unordered_map<Point, MidPoint*>& point_to_midpoint,
    std::unordered_set<std::shared_ptr<Cone>>& discarded_cones
  ); 

/**
 * @brief Find the nearest midpoint to a target point within a tolerance
 * 
 * @param target The target point to find the nearest midpoint for
 * @param map Map of points to midpoints for quick access
 * @param tolerance The maximum distance to consider a point as "near"
 * @return MidPoint* Pointer to the nearest midpoint, or nullptr if none found
 */
  MidPoint* find_nearest_point(
    const Point& target,
    const std::unordered_map<Point, MidPoint*>& map
  );
  
  /**
   * @brief Resets the path periodically to avoid long-term drift or degradation
   *
   * If use_sliding_window_ is enabled, updates mid_points_ 
   * to allow the path to be reconstructed again. 
   *
   * @param cone_array   Vector of cones representing the current detected cones.
   * @return int         Updated maximum number of points for the path.
   */
  int reset_path(const std::vector<Cone>& cone_array);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_