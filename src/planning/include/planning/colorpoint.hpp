#ifndef SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common_lib/structures/cone.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;
using Cone = common_lib::structures::Cone;
using Color = common_lib::competition_logic::Color;

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
   * @brief Constructs a Colorpoint with a Point and two cones.
   * @param pt The Point
   * @param c1 The first cone
   * @param c2 The second cone
   */
  Colorpoint(const Point& pt, Cone c1, Cone c2) : point(pt), cone1(c1), cone2(c2) {}

  /**
   * @brief Extracts and classifies all cones from a sequence of colorpoints.
   *
   * This method processes a vector of colorpoints representing the racing path
   * and classifies each cone as either left (blue) or right (yellow) based on
   * the path direction using cross product calculations.
   *
   * @param colorpoints Vector of colorpoints defining the path
   * @param yellow_cones Output vector for right-side (yellow) cones
   * @param blue_cones Output vector for left-side (blue) cones
   */
  static void extract_cones(std::vector<Colorpoint>& colorpoints, std::vector<Cone>& yellow_cones,
                            std::vector<Cone>& blue_cones);

  Point point;  ///< The path point (midpoint between cones)
  Cone cone1;   ///< First cone associated with this path point
  Cone cone2;   ///< Second cone associated with this path point

private:

  /**
   * @brief Checks if a colorpoint's cones are already correctly colored.
   *
   * @param colorpoint The colorpoint to check
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   * @return true if cones are already colored correctly, false otherwise
   */
  static bool is_colored_right(const Colorpoint& colorpoint, std::vector<Cone>& yellow_cones,
                                    std::vector<Cone>& blue_cones);

  /**
   * @brief Colors cones using cross product to determine left/right orientation.
   *
   * @param colorpoint Current colorpoint to color
   * @param next_colorpoint Next colorpoint (defines path direction)
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void color_cones(Colorpoint& colorpoint, const Colorpoint& next_colorpoint,
                          std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones);

  /**
   * @brief Colors the cones of the last colorpoint in the path.
   *
   * @param colorpoints Vector of all colorpoints
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void color_last_point(std::vector<Colorpoint>& colorpoints,
                               std::vector<Cone>& yellow_cones, std::vector<Cone>& blue_cones);

  /**
   * @brief Helper function to add cones based on a reference cone's color.
   *
   * @param reference_cone The cone with a known color to use as reference
   * @param matching_cone The cone that should have the same color as reference
   * @param other_cone The cone that should have the opposite color
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void add_cones_by_reference(const Cone& reference_cone, const Cone& matching_cone,
                                          const Cone& other_cone, std::vector<Cone>& yellow_cones,
                                          std::vector<Cone>& blue_cones);
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_