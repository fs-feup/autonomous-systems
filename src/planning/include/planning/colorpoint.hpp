#ifndef SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/path_point.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;
using Cone = common_lib::structures::Cone;
using Color = common_lib::competition_logic::Color;
using PathPoint = common_lib::structures::PathPoint;

/**
 * @class Colorpoint
 * @brief Path point with two boundary cones, provides cone classification into left/right
 * boundaries.
 */
class Colorpoint {
public:
  /**
   * @brief Default constructor.
   */
  Colorpoint() = default;

  /**
   * @brief Constructs a Colorpoint with a Point and two boundary cones.
   * @param pt The Point
   * @param c1 The first boundary cone.
   * @param c2 The second boundary cone.
   */
  Colorpoint(const Point& pt, Cone c1, Cone c2) : point(pt), cone1(c1), cone2(c2) {}

  /**
   * @brief Extracts and classifies all cones from a sequence of colorpoints.
   *
   * This method processes a vector of colorpoints representing the racing path
   * and classifies each cone as either left (blue) or right (yellow) based on
   * the path direction using cross product calculations.
   *
   * @param colorpoints Vector of colorpoints defining the path.
   * @param yellow_cones Output vector for right-side (yellow) cones.
   * @param blue_cones Output vector for left-side (blue) cones.
   */
  static void extract_cones(std::vector<Colorpoint>& colorpoints,
                            std::vector<PathPoint>& yellow_cones,
                            std::vector<PathPoint>& blue_cones);

  Point point;  ///< The path point (midpoint between cones).
  Cone cone1;   ///< First cone associated with this path point.
  Cone cone2;   ///< Second cone associated with this path point.

private:
  /**
   * @brief Classifies the cones of a single colorpoint using the direction to the next point.
   *
   * @param colorpoint The current colorpoint whose cones are to be classified.
   * @param next_colorpoint The following colorpoint, used to determine path direction.
   * @param yellow_cones Output vector for the right-side (yellow) cones.
   * @param blue_cones Output vector for the left-side (blue) cones.
   */
  static void color_cones(Colorpoint& colorpoint, const Colorpoint& next_colorpoint,
                          std::vector<PathPoint>& yellow_cones, std::vector<PathPoint>& blue_cones);

  /**
   * @brief Classifies the cones of the last colorpoint using a shared cone from the previous one.
   *
   * @param colorpoints The full sequence of colorpoints. Must contain at least 2 elements.
   * @param yellow_cones Output vector for the right-side (yellow) cones.
   * @param blue_cones Output vector for the left-side (blue) cones.
   */
  static void color_last_point(std::vector<Colorpoint>& colorpoints,
                               std::vector<PathPoint>& yellow_cones,
                               std::vector<PathPoint>& blue_cones);
  /**
   * @brief Classifies a cone pair using a reference cone whose color is already known.
   *
   * @param reference_cone A cone whose color is already determined, used as the classification
   * anchor.
   * @param matching_cone The cone in the last point that corresponds to the reference cone.
   * @param other_cone The remaining cone in the last point, assigned the opposite color.
   * @param yellow_cones Output vector for the right-side (yellow) cones.
   * @param blue_cones Output vector for the left-side (blue) cones.
   */
  static void add_cones_by_reference(const Cone& reference_cone, Cone& matching_cone,
                                     Cone& other_cone, std::vector<PathPoint>& yellow_cones,
                                     std::vector<PathPoint>& blue_cones);
  /**
   * @brief Assigns colors to a cone pair and appends them to the output vectors.
   *
   * @param yellow_cone The cone to be marked as yellow (right boundary).
   * @param blue_cone The cone to be marked as blue (left boundary).
   * @param yellow_cones Output vector for the right-side (yellow) cones.
   * @param blue_cones Output vector for the left-side (blue) cones.
   */
  static void color_pair_of_cones(Cone& yellow_cone, Cone& blue_cone,
                                  std::vector<PathPoint>& yellow_cones,
                                  std::vector<PathPoint>& blue_cones);
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_COLORPOINt_HPP_