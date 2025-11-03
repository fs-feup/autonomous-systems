#ifndef SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/midpoint.hpp"
#include "common_lib/structures/pose.hpp"
#include "config/midpoint_generator_config.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using DT = CGAL::Delaunay_triangulation_2<K>;
using Point = K::Point_2;
using Vertex_handle = DT::Vertex_handle;

using Pose = common_lib::structures::Pose;
using Cone = common_lib::structures::Cone;
using Midpoint = common_lib::structures::Midpoint;

/**
 * @class MidpointGenerator
 * @brief Generates midpoints between cones using Delaunay triangulation.
 *
 * The algorithm performs the following main steps:
 * - Filters cones using a sliding window based on the vehicle’s position (if enabled).
 * - Builds a Delaunay triangulation from the cone coordinates.
 * - Creates midpoints for valid cone pairs (within configured distance limits).
 * - Connects midpoints belonging to the same triangle.
 */
class MidpointGenerator {
public:
  /**
   * @brief Default constructor.
   */
  MidpointGenerator() = default;

  /**
   * @brief Constructs a new MidpointGenerator with a specific configuration.
   *
   * @param config Configuration object defining thresholds and filtering behavior.
   */
  explicit MidpointGenerator(const MidpointGeneratorConfig& config) : config_(config) {}

  /**
   * @brief Generates midpoints from a set of cones.
   *
   * This method filters cones (if enabled), computes the Delaunay triangulation,
   * and generates valid midpoints between cone pairs that meet distance constraints.
   *
   * @param cone_array Input vector of cone objects.
   * @param rebuild_all_midpoints Whether to use all cones
   * @return Reference to the vector containing all generated midpoints.
   */
  std::vector<std::shared_ptr<Midpoint>>& generate_midpoints(const std::vector<Cone>& cone_array,
                                                             bool rebuild_all_midpoints);

  /**
   * @brief Returns the current set of Delaunay edges used for visualization.
   *
   * @return A const reference to the vector of triangulated edges.
   */
  const std::vector<std::pair<Point, Point>>& get_triangulations() const;

  /**
   * @brief Updates the vehicle pose for dynamic filtering.
   *
   * This pose is used to center the sliding window for filtering relevant cones.
   *
   * @param vehicle_pose The current vehicle pose.
   */
  void set_vehicle_pose(const Pose& vehicle_pose);

private:
  // Stores Delaunay triangulation edges for visualization and debugging.
  std::vector<std::pair<Point, Point>> triangulations_;

  // Current vehicle pose.
  Pose vehicle_pose_;

  // Vector containing all generated midpoints.
  std::vector<std::shared_ptr<Midpoint>> midpoints_;

  MidpointGeneratorConfig config_;

  /**
   * @brief Filters input cones based on proximity to the vehicle pose.
   *
   *
   * @param cone_array Input array of detected cones.
   * @param filtered_cones Output vector containing cones after filtering.
   * @param rebuild_all_midpoints Whether to reset filtering and include all cones.
   */
  void filter_cones(const std::vector<Cone>& cone_array,
                    std::vector<std::shared_ptr<Cone>>& filtered_cones, bool rebuild_all_midpoints);

  /**
   * @brief Processes a single edge of a Delaunay triangle and creates its midpoint if valid.
   *
   * This function verifies the corresponding cones for the edge, checks distance constraints,
   * and either creates a new midpoint or reuses an existing one to prevent duplication.
   *
   * @param va First vertex of the triangle edge.
   * @param vb Second vertex of the triangle edge.
   * @param filtered_cones Vector of all filtered cones.
   * @param segment_to_midpoint Map used to ensure unique midpoint creation per cone pair.
   * @return A shared pointer to the created or existing midpoint, or nullptr if invalid.
   */
  std::shared_ptr<Midpoint> process_triangle_edge(
      const Vertex_handle& va, const Vertex_handle& vb,
      std::vector<std::shared_ptr<Cone>>& filtered_cones,
      std::map<std::pair<int, int>, std::shared_ptr<Midpoint>>& segment_to_midpoint);

  /**
   * @brief Connects midpoints that belong to the same Delaunay triangle.
   *
   * @param triangle_midpoints Array of up to three midpoints from one triangle.
   */
  void connect_triangle_midpoints(
      const std::array<std::shared_ptr<Midpoint>, 3>& triangle_midpoints);

  /**
   * @brief Finds a cone in a vector based on its position coordinates.
   *
   * @param cones Reference to a vector of shared pointers to `Cone` objects.
   * @param x The x-coordinate of the cone to find.
   * @param y The y-coordinate of the cone to find.
   * @return The index of the cone with the matching position, or -1 if not found.
   */
  int find_cone(std::vector<std::shared_ptr<Cone>>& cones, double x, double y);
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_
