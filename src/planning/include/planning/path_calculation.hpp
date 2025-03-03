#ifndef SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <cmath>
#include <map>
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


  struct MidPoint;

  struct Vertex {
    Point point;
    std::vector<MidPoint*> neighbors;

    bool operator==(const Vertex& other) const {
        return (point.x() == other.point.x()) && (point.y() == other.point.y());
    }
  };

  struct VertexHash {
    static constexpr double equality_tolerance = 0.1;  // Tolerance for quantization

    std::size_t operator()(const Vertex& v) const noexcept {
        // Quantize function rounds values based on tolerance
        auto quantize = [](double value, double tolerance) { return std::round(value / tolerance); };

        std::size_t x_hash = std::hash<double>()(quantize(v.point.x(), equality_tolerance));
        std::size_t y_hash = std::hash<double>()(quantize(v.point.y(), equality_tolerance));

        return x_hash ^ (y_hash << 1);  // Combine hashes
    }
};
  

  struct MidPoint {
    Point point;
    Vertex* v1;
    Vertex* v2;
    std::vector<MidPoint*> close_points;

    bool operator==(const MidPoint& other) const {
        return (point.x() == other.point.x()) && (point.y() == other.point.y());
    }

  };

  double dfs_cost(int depth, MidPoint *previous, MidPoint *current, double maxcost);

  std::vector<PathPoint> no_coloring_planning(std::vector<Cone>& cone_array,
                                              common_lib::structures::Pose pose);


  std::vector<PathPoint> skidpad_path(std::vector<Cone>& cone_array,
                                      common_lib::structures::Pose pose);
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_PATH_CALCULATION_HPP_
