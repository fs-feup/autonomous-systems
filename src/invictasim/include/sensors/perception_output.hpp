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
 * and applies error injection including detection probability, motion skew, and noise.
 */
class PerceptionOutput : public Sensor {
public:

  /**
   * @brief Structure to hold transformed cone data
   */
  struct TransformedCone {
    common_lib::structures::Cone original_cone;  ///< Original global cone
    double x_local = 0.0;           ///< X coordinate in local LiDAR frame
    double y_local = 0.0;           ///< Y coordinate in local LiDAR frame
    double z_local = 0.0;           ///< Z coordinate in local LiDAR frame
    double range_3d = 0.0;          ///< 3D slant range
    double elevation_angle = 0.0;   ///< Elevation angle (vertical FOV)
    double azimuth_angle = 0.0;     ///< Azimuth angle (horizontal FOV)
    double detection_probability = 0.0;  ///< Probability of detection
    bool is_visible = false;        ///< Whether cone passes visibility filters
    double x_skew = 0.0;            ///< X coordinate after motion skew
    double y_skew = 0.0;            ///< Y coordinate after motion skew
    double z_skew = 0.0;            ///< Z coordinate after motion skew
    double x_noisy = 0.0;           ///< X coordinate with noise applied
    double y_noisy = 0.0;           ///< Y coordinate with noise applied
    double z_noisy = 0.0;           ///< Z coordinate with noise applied
  };

  /**
   * @brief Construct a new PerceptionOutput object
   * @param config_path Path to the perception.yaml configuration file
   */
  explicit PerceptionOutput(const std::string& config_path);

  /**
   * @brief Apply perception error modeling to global cone coordinates
   * Transforms cones to LiDAR local frame and applies error injection including 
   * detection probability, motion skew, and noise.
   *
   * @param cones Vector of cones in global coordinates
   * @param vehicle_pose Current vehicle pose (position and yaw)
   * @param vehicle_velocities Current vehicle velocities
   * @return Vector of transformed cones with error modeling applied
   */
  std::vector<TransformedCone> perception_error(
      const std::vector<common_lib::structures::Cone>& cones,
      const common_lib::structures::Pose& vehicle_pose,
      const common_lib::structures::Velocities& vehicle_velocities);

private:
  // LiDAR sensor parameters
  double height_;                    // Height of LiDAR relative to vehicle origin (meters)
  double max_range_;                 // Maximum detection range (meters)
  double horizontal_fov_angle_;      // Horizontal field of view (radians)
  double vertical_fov_angle_;        // Vertical field of view (radians)
  double angular_velocity_;          // LiDAR angular velocity (rad/s)
  double detection_probability_alpha_;  // Alpha parameter for detection sigmoid
  double noise_std_dev_base_;        // Base standard deviation for noise (meters)
  bool noise_scales_with_range_;     // Whether noise scales with range
  double noise_range_scaling_;       // Scaling factor for range-dependent noise

};
