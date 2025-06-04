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
  Eigen::Matrix4f last_icp_transform_ = Eigen::Matrix4f::Identity();
  bool has_icp_transform_ = false;
  bool path_aligned_ = false;



  // Anchor point for the path, to avoid calculating the path from the position of the car
  common_lib::structures::Pose anchor_point_;
  bool anchor_point_set_ = false;

public:
  /**
   * @brief MidPoint struct represents a potential path point with connections
   */
  struct MidPoint {
    Point point;
    std::vector<MidPoint *> close_points;
    Cone* cone1;
    Cone* cone2;
    bool valid = true;

    bool operator==(const MidPoint& other) const {
      return point == other.point && cone1 == other.cone1 && cone2 == other.cone2;
    }

  };

  struct point_hash {
    std::size_t operator()(const Point& p) const {
        auto h1 = std::hash<double>()(p.x());
        auto h2 = std::hash<double>()(p.y());
        return h1 ^ (h2 << 1);
    }
  };

  struct pair_hash {
    std::size_t operator()(const std::pair<Point, Point>& p) const {
        auto h1 = point_hash{}(p.first);
        auto h2 = point_hash{}(p.second);
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
   * @brief Creates midpoints from triangulation of cone positions
   *
   * @param cone_array The array of cones
   * @return std::vector<std::unique_ptr<MidPoint>> Vector of midpoints
   */
  std::vector<std::unique_ptr<MidPoint>> createMidPointsFromTriangulation(
      const std::vector<Cone>& cone_array);

  /**
   * @brief Establishes connections between close midpoints
   *
   * @param mid_points Vector of midpoints to connect
   */
  void establishMidPointConnections(const std::vector<std::unique_ptr<MidPoint>>& mid_points);

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
   * @brief Find the second point for the path based on the first point and projection
   *
   * @param mid_points Vector of available midpoints
   * @param first The first point of the path
   * @param projected The projected point based on vehicle orientation
   * @param anchor_pose The anchor pose for reference
   * @return MidPoint* The second point for the path
   */
  MidPoint* findSecondPoint(const std::vector<std::unique_ptr<MidPoint>>& mid_points,
                            const MidPoint* first, const MidPoint& projected,
                            const common_lib::structures::Pose& anchor_pose);

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
  std::vector<PathPoint> skidpad_path(std::vector<Cone>& cone_array,
                                      common_lib::structures::Pose pose);

  std::vector<PathPoint> getGlobalPath() const;


 MidPoint* find_nearest_point(
  const Point& target,
  const std::unordered_map<Point, MidPoint*, point_hash>& map,
  double tolerance);


MidPoint* find_matching_midpoint_pcl(const Point& query, 
  const pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree, 
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  const std::vector<MidPoint*>& index_map,
  double radius); 

};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_