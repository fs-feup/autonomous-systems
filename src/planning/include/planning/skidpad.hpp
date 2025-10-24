#ifndef SRC_PLANNING_INCLUDE_PLANNING_SKIDPAD_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_SKIDPAD_HPP_

#include <vector>
#include <utility>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/cone.hpp"
#include "config/skidpad_config.hpp"


using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;
using Cone = common_lib::structures::Cone;

/**
 * @brief class that defines the skidpad path algorithm
 *
 */
class Skidpad {
public:

  Skidpad() = default;

  explicit Skidpad(SkidpadConfig config) : config_(config) {}

  /**
   * @brief Generate a path for skidpad course
   *
   * @param cone_array The array of cones representing the track
   * @param pose The current pose of the vehicle
   * @return std::vector<PathPoint> The generated path
   */
  std::vector<PathPoint> skidpad_path(const std::vector<Cone>& cone_array,
                                      common_lib::structures::Pose pose);
private:

  SkidpadConfig config_;

  bool skidpad_data_loaded_ = false;
  std::vector<std::pair<double, double>> reference_cones_;
  std::vector<PathPoint> hardcoded_path_;
  std::vector<PathPoint> predefined_path_;

  /**
   * @brief Aligns detected cones with reference cones using ICP
   * 
   * @param cone_array Detected cones
   * @return Eigen::Matrix4f Transformation matrix, or identity if ICP fails
   */
  Eigen::Matrix4f align_cones_with_icp(const std::vector<Cone>& cone_array);

  /**
   * @brief Finds the closest point index in a path to a given pose
   * 
   * @param path Path to search
   * @param pose Reference pose
   * @return size_t Index of closest point
   */
  size_t find_closest_path_index(
      const std::vector<PathPoint>& path,
      const common_lib::structures::Pose& pose);
};

#endif // SRC_PLANNING_INCLUDE_PLANNING_SKIDPAD_HPP_