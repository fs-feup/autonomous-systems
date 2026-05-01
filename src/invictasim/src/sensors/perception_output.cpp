#include "sensors/perception_output.hpp"

#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <fstream>

PerceptionOutput::PerceptionOutput(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node lidar = config["lidar_sensor"];

  height_ = lidar["height"].as<double>();
  max_range_ = lidar["max_range"].as<double>();
  horizontal_fov_angle_ = lidar["horizontal_fov_angle"].as<double>();
  vertical_fov_angle_ = lidar["vertical_fov_angle"].as<double>();
  angular_velocity_ = lidar["angular_velocity"].as<double>();
  detection_probability_alpha_ = lidar["detection_probability_alpha"].as<double>();
  noise_std_dev_base_ = lidar["noise_std_dev_base"].as<double>();
  noise_scales_with_range_ = lidar["noise_scales_with_range"].as<bool>();
  noise_range_scaling_ = lidar["noise_range_scaling"].as<double>();
}

std::vector<PerceptionOutput::TransformedCone> PerceptionOutput::perception_error(
    const std::vector<common_lib::structures::Cone>& cones,
    const common_lib::structures::Pose& vehicle_pose,
    const common_lib::structures::Velocities& vehicle_velocities) {
  std::vector<TransformedCone> transformed_cones;

  double yaw_cos = std::cos(vehicle_pose.orientation);
  double yaw_sin = std::sin(vehicle_pose.orientation);

  // Transform each cone
  for (const auto& cone : cones) {
    TransformedCone transformed;
    transformed.original_cone = cone;

    // Translate to vehicle-relative coordinates
    double dx = cone.position.x - vehicle_pose.position.x;
    double dy = cone.position.y - vehicle_pose.position.y;
    double dz = 0.0;
    
    // Rotate to vehicle frame based on yaw angle (psi)
    // [ cos(psi)  sin(psi) ]
    // [-sin(psi)  cos(psi) ]
    double x_vehicle = yaw_cos * dx + yaw_sin * dy;
    double y_vehicle = -yaw_sin * dx + yaw_cos * dy;

    // Transform to LiDAR local frame (accounting for height offset)
    transformed.x_local = x_vehicle;
    transformed.y_local = y_vehicle;
    transformed.z_local = dz - height_;

    // Calculate 3D slant range
    transformed.range_3d =
        std::sqrt(std::pow(transformed.x_local, 2) + std::pow(transformed.y_local, 2) +
                  std::pow(transformed.z_local, 2));

    // Calculate elevation angle (vertical FOV angle)
    double horizontal_distance =
        std::sqrt(std::pow(transformed.x_local, 2) + std::pow(transformed.y_local, 2));
    transformed.elevation_angle =
        std::atan2(transformed.z_local, horizontal_distance);

    // Calculate azimuth angle (horizontal FOV angle)
    transformed.azimuth_angle = std::atan2(transformed.y_local, transformed.x_local);

    bool is_visible = true;

    // Check elevation angle (V-FOV)
    if (is_visible) {
      double v_fov_half = vertical_fov_angle_ / 2.0;
      if (std::abs(transformed.elevation_angle) > v_fov_half) {
        is_visible = false;
      }
    }

    // Check azimuth angle (H-FOV)
    if (is_visible) {
      double h_fov_half = horizontal_fov_angle_ / 2.0;
      if (std::abs(transformed.azimuth_angle) > h_fov_half) {
        is_visible = false;
      }
    }

    transformed.is_visible = is_visible;

    if (transformed.is_visible) {
      // Inverted sigmoid function: P_det(r_3D) = 1 / (1 + e^(alpha * (r_3D - R_max)))
      double exponent =
          detection_probability_alpha_ * (transformed.range_3d - max_range_);
      // Clamp exponent to avoid overflow
      exponent = std::clamp(exponent, -100.0, 100.0);
      transformed.detection_probability = 1.0 / (1.0 + std::exp(exponent));

      double scan_angle = transformed.azimuth_angle;
      if (scan_angle < 0) {
        scan_angle += 2.0 * M_PI;
      }
      double delta_t_skew = scan_angle / angular_velocity_;

      // Apply velocity-based displacement
      transformed.x_skew =
          transformed.x_local + vehicle_velocities.velocity_x * delta_t_skew;
      transformed.y_skew =
          transformed.y_local + vehicle_velocities.velocity_y * delta_t_skew;
      transformed.z_skew = transformed.z_local;

      // Calculate noise standard deviation
      double sigma = noise_std_dev_base_;
      if (noise_scales_with_range_) {
        sigma +=
            noise_range_scaling_ * transformed.range_3d;
      }

      // Apply Gaussian noise to skew-corrected coordinates
      transformed.x_noisy = transformed.x_skew + gaussian_noise(sigma);
      transformed.y_noisy = transformed.y_skew + gaussian_noise(sigma);
      transformed.z_noisy = transformed.z_skew + gaussian_noise(sigma);

      transformed_cones.push_back(transformed);
    }
  }

  return transformed_cones;
}
