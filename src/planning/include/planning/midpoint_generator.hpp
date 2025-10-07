#ifndef SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/midpoint.hpp"
#include "config/path_calculation_config.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using DT = CGAL::Delaunay_triangulation_2<K>;
using Point = K::Point_2;

//using Finite_edges_iterator = DT::Finite_edges_iterator;
using Vertex_handle = DT::Vertex_handle;
using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;
using Cone = common_lib::structures::Cone;
using Midpoint = common_lib::structures::Midpoint;

/**
 * @brief class that defines the skidpad path algorithm
 *
 */
class MidpointGenerator {
private:
  /* Stores the edges of the Delaunay triangulation as pairs of points,
  used for visualization (each pair represents one edge of a triangle).*/
  std::vector<std::pair<Point, Point>> triangulations_;
  Pose vehicle_pose_;
  std::vector<std::shared_ptr<Midpoint>> midpoints_;
  MidpointGeneratorConfig config_;


  void filter_cones(const std::vector<Cone>& cone_array,
                    std::vector<std::shared_ptr<Cone>>& filtered_cones,
                    bool should_reset);

  /**
   * @brief Processes a single edge of a Delaunay triangle
   * 
   * @param va First vertex of the edge
   * @param vb Second vertex of the edge
   * @param filtered_cones Vector of all cones
   * @param segment_to_midpoint Map to track unique midpoints
   * @return std::shared_ptr<Midpoint> Created or existing midpoint, or nullptr if invalid
   */
  std::shared_ptr<Midpoint> process_triangle_edge(
      const Vertex_handle& va,
      const Vertex_handle& vb,
      std::vector<std::shared_ptr<Cone>>& filtered_cones,
      std::map<std::pair<int, int>, std::shared_ptr<Midpoint>>& segment_to_midpoint);


  /**
   * @brief Connects midpoints that share the same triangle
   * 
   * @param triangle_midpoints Array of 3 midpoints from a triangle
   */
  void connect_triangle_midpoints(
      const std::array<std::shared_ptr<Midpoint>, 3>& triangle_midpoints);
  
  

public:
    MidpointGenerator() = default;
     /**
     * @brief Construct a new DelaunayMidpointGenerator
     * 
     * @param config Configuration for distance thresholds and filtering
     */
    explicit MidpointGenerator(const MidpointGeneratorConfig& config): config_(config) {}

    std::vector<std::shared_ptr<Midpoint>>& generate_midpoints(std::vector<Cone>& cone_array, bool should_reset); 

    const std::vector<std::pair<Point, Point>>& get_triangulations() const;

    /**
     * @brief Updates the vehicle pose
     *
     * @param vehicle_pose The current vehicle pose
     */
    void set_vehicle_pose(const Pose& vehicle_pose);

};

#endif // SRC_PLANNING_INCLUDE_PLANNING_DELAUNY_MIDPOINT_GENERATOR_HPP_