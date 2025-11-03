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
using Color = common_lib::competition_logic::Color;

/**
 * @class Colorpoint
 * @brief Represents a point on the racing path with two associated cones.
 * 
 * A Colorpoint contains a path point (typically the midpoint between two cones)
 * and references to the two cones that define the track boundaries at that location.
 * This class provides functionality to classify cones as left (blue) or right (yellow)
 * based on the path direction.
 */
class Colorpoint {
public:
  /**
   * @brief Default constructor.
   */
  Colorpoint() = default;

  /**
   * @brief Constructs a Colorpoint with a path point and two cones.
   * @param pt The point on the path (typically midpoint between cones)
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
  static void extract_cones(std::vector<Colorpoint>& colorpoints,
                           std::vector<Cone>& yellow_cones, 
                           std::vector<Cone>& blue_cones);

  Point point;  ///< The path point (midpoint between cones)
  Cone cone1;   ///< First cone associated with this path point
  Cone cone2;   ///< Second cone associated with this path point

private:
  /**
   * @brief Adds a cone to the appropriate vector if it's not a duplicate.
   * 
   * @param cone The cone to add
   * @param cones Vector to add the cone to
   */
  static void add_cone(const Cone& cone, std::vector<Cone>& cones);

  /**
   * @brief Checks if a colorpoint's cones are already correctly colored.
   * 
   * @param colorpoint The colorpoint to check
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   * @return true if cones are already colored correctly, false otherwise
   */
  static bool is_colored_right(Colorpoint& colorpoint, 
                               std::vector<Cone>& yellow_cones,
                               std::vector<Cone>& blue_cones);

  /**
   * @brief Colors cones using cross product to determine left/right orientation.
   * 
   * Uses the cross product method to determine which cone is on the left (blue)
   * and which is on the right (yellow) based on the path direction from the
   * current colorpoint to the next colorpoint.
   * 
   * The cross product is calculated as: v × w = v_x * w_y - v_y * w_x
   * - Positive cross product: cone is on the left (blue)
   * - Negative cross product: cone is on the right (yellow)
   * 
   * @param colorpoint Current colorpoint to color
   * @param next_colorpoint Next colorpoint (defines path direction)
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void color_cones(Colorpoint& colorpoint, 
                         const Colorpoint& next_colorpoint,
                         std::vector<Cone>& yellow_cones, 
                         std::vector<Cone>& blue_cones);

  /**
   * @brief Colors the cones of the last colorpoint in the path.
   * 
   * The last point requires special handling since there's no "next" point to
   * define the path direction. This method uses the relationship between the
   * second-to-last and last colorpoint's cones to infer the correct coloring.
   * 
   * @param colorpoints Vector of all colorpoints
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void color_last_point(std::vector<Colorpoint>& colorpoints,
                               std::vector<Cone>& yellow_cones,
                               std::vector<Cone>& blue_cones);

  /**
   * @brief Helper function to add cones based on a reference cone's color.
   * 
   * Adds two cones to the appropriate color vectors based on the color of
   * a reference cone. The matching cone gets the same color as the reference,
   * while the other cone gets the opposite color.
   * 
   * @param reference_cone The cone with a known color to use as reference
   * @param matching_cone The cone that should have the same color as reference
   * @param other_cone The cone that should have the opposite color
   * @param yellow_cones Vector of yellow (right-side) cones
   * @param blue_cones Vector of blue (left-side) cones
   */
  static void add_cones_by_reference(const Cone& reference_cone,
                                     Cone& matching_cone,
                                     Cone& other_cone,
                                     std::vector<Cone>& yellow_cones,
                                     std::vector<Cone>& blue_cones);
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_COLORPOINT_HPP_