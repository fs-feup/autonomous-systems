#ifndef SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_

#include <memory>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;
using Cone = common_lib::structures::Cone;

/**
 * @class Colorpoint
 * @brief Represents a point with its associated left and right cones.
 *
 * This class stores a point along with references to the two cones that
 * define it.
 *
 *
 *
 *
 *
 * It also provides
 * static functionality to extract all colored cones from a collection of
 * ColorPoints.
 */
class Colorpoint {
public:
  /**
   * @brief Default constructor.
   */
  Colorpoint() = default;
  /**
   * @brief Construct a ColorPoint from a Point and two cones.
   * @param pt The point.
   * @param c1 First cone (typically one side of track).
   * @param c2 Second cone (typically opposite side of track).
   */
  Colorpoint(const Point& pt, Cone c1, Cone c2) : point(pt), cone1(c1), cone2(c2) {}

  Point point;
  Cone cone1;
  Cone cone2;

  /**
   * @brief Extract all unique cones from a vector of ColorPoints.
   *
   * Iterates through the ColorPoints and collects all associated cones,
   * removing duplicates. Useful for visualization or analysis of which
   * cones are being used in a path.
   *
   * @param color_points Vector of ColorPoints to extract cones from.
   * @return Vector of unique cone pointers found in the ColorPoints.
   */
//   static void extract_cones(const std::vector<Colorpoint>& color_points,
//                             std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones);
private:


// 
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_