#pragma once

#include <vector>
#include <cmath>
#include <memory>
#include <string>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/velocities.hpp"
#include "sensors/sensors_base.hpp"

/**
 * @brief Perception output class for handling LiDAR perception with error modeling
 * 
 * This class transforms global cone coordinates into the LiDAR's local reference frame
 * and applies error injection including detection probability and noise.
 */
class SimulatedPerception : public Sensor {
public:

  /**
   * @brief Construct a new SimulatedPerception object
   * @param config_path Path to the perception.yaml configuration file
   */
  explicit SimulatedPerception(const std::string& config_path);

  /**
   * @brief Apply perception error modeling to global cone coordinates
   * Transforms cones to LiDAR local frame and applies error injection including 
   * detection probability and noise.
   *
   * @param cones Vector of cones in global coordinates
   * @param vehicle_pose Current vehicle pose (position and yaw)
   * @param vehicle_velocities Current vehicle velocities
   * @return Vector of transformed cones with error modeling applied
   */
  std::vector<common_lib::structures::Cone> perception_error(
      const std::vector<common_lib::structures::Cone>& cones,
      const common_lib::structures::Pose& vehicle_pose,
      const common_lib::structures::Velocities& vehicle_velocities);

private:
  // LiDAR sensor parameters
  double height_;                       // Height of LiDAR relative to vehicle origin (meters)
  double max_range_;                    // Maximum detection range (meters)
  double horizontal_fov_angle_;         // Horizontal field of view (radians)
  double vertical_fov_angle_;           // Vertical field of view (radians)
  double detection_probability_alpha_;  // Alpha parameter for detection sigmoid
  double noise_std_dev_base_;           // Base standard deviation for noise (meters)
  bool noise_scales_with_range_;        // Whether noise scales with range
  double noise_range_scaling_;          // Scaling factor for range-dependent noise
  double mounting_pitch_;               // Mounting pitch angle of the LiDAR (radians)
};
