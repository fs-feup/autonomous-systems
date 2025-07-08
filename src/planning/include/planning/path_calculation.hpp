#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <map>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "config/path_calculation_config.hpp"
#include "rclcpp/rclcpp.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using DT = CGAL::Delaunay_triangulation_2<K>;
using Point = K::Point_2;

using Vertex_handle = DT::Vertex_handle;
using Finite_edges_iterator = DT::Finite_edges_iterator;

using Cone = common_lib::structures::Cone;
using PathPoint = common_lib::structures::PathPoint;

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

  // Anchor point for the path, to avoid calculating the path from the position of the car
  common_lib::structures::Pose anchor_point_;
  bool anchor_point_set_ = false;

public:
  /**
   * @brief MidPoint struct represents a potential path point with connections
   */
  struct MidPoint {
    Point point;
    std::vector<MidPoint*> close_points;
    Cone* cone1;
    Cone* cone2;
    bool valid = true;
  };

  struct PointHash {
    std::size_t operator()(const Point& p) const {
      auto h1 = std::hash<double>()(p.x());
      auto h2 = std::hash<double>()(p.y());
      return h1 ^ (h2 << 1);
    }
  };

  struct PairHash {
    std::size_t operator()(const std::pair<Point, Point>& p) const {
      auto h1 = PointHash{}(p.first);
      auto h2 = PointHash{}(p.second);
      return h1 ^ (h2 << 1);
    }
  };

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
   * @brief Process an array of cones to generate a local path.
   *
   * This function processes an array of cones representing a track and
   * generates a local path by selecting positions based on certain criteria.
   *
   * @param cone_array Pointer to the array of cones representing the track.
   * @return A vector of pointers to PathPoint objects representing the generated
   * path.
   * @details The function utilizes Delaunay triangulation (CGAL) and
   * direction-based selection of positions to create a meaningful local path.
   */
  std::vector<PathPoint> process_delaunay_triangulations(
      std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones) const;

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
   * @brief Generate a path from cone array without color information
   *
   * @param cone_array The array of cones representing the track
   * @param pose The current pose of the vehicle
   * @return std::vector<PathPoint> The generated path
   */
  std::vector<PathPoint> no_coloring_planning(std::vector<Cone>& cone_array,
                                              common_lib::structures::Pose pose);

  /**
   * @brief Updates the anchor point if not already set
   *
   * @param pose The current vehicle pose
   */
  void updateAnchorPoint(const common_lib::structures::Pose& pose);

  /**
   * @brief Finds the first and second points to start the path
   *
   * @param mid_points Vector of available midpoints
   * @param anchor_pose The anchor pose for reference
   * @return std::pair<MidPoint*, MidPoint*> First and second points for the path
   */
  std::pair<MidPoint*, MidPoint*> findPathStartPoints(
      const std::vector<std::unique_ptr<MidPoint>>& mid_points,
      const common_lib::structures::Pose& anchor_pose);

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
  std::vector<PathPoint> calculate_trackdrive(std::vector<Cone>& cone_array,
                                              common_lib::structures::Pose pose);

  std::vector<PathPoint> getGlobalPath() const;

  void createMidPoints(std::vector<Cone>& cone_array,
                       std::vector<std::unique_ptr<MidPoint>>& midPoints,
                       std::unordered_map<MidPoint*, std::vector<Point>>& triangle_points);

  void connectMidPoints(const std::vector<std::unique_ptr<MidPoint>>& midPoints,
                        const std::unordered_map<MidPoint*, std::vector<Point>>& triangle_points);

  void selectInitialPath(std::vector<Point>& path,
                         const std::vector<std::unique_ptr<MidPoint>>& midPoints,
                         const common_lib::structures::Pose& pose,
                         const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
                         std::unordered_set<MidPoint*>& visited_midpoints,
                         std::unordered_set<Cone*>& discarded_cones);

  void extendPath(std::vector<Point>& path, const std::vector<std::unique_ptr<MidPoint>>& midPoints,
                  const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
                  std::unordered_set<MidPoint*>& visited_midpoints,
                  std::unordered_set<Cone*>& discarded_cones, int max_points);

  void discard_cones_along_path(
      const std::vector<Point>& path, const std::vector<std::unique_ptr<MidPoint>>& midPoints,
      const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
      std::unordered_set<Cone*>& discarded_cones);

  MidPoint* find_nearest_point(const Point& target,
                               const std::unordered_map<Point, MidPoint*, PointHash>& map,
                               double tolerance);

  std::pair<Point, Point> ordered_segment(const Point& a, const Point& b);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_