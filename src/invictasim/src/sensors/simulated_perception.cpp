#include "sensors/simulated_perception.hpp"

#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <fstream>

SimulatedPerception::SimulatedPerception(const std::string& config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  YAML::Node lidar = config["lidar_sensor"];

  height_ = lidar["height"].as<double>();
  max_range_ = lidar["max_range"].as<double>();
  horizontal_fov_angle_ = lidar["horizontal_fov_angle"].as<double>() * M_PI / 180.0;
  vertical_fov_angle_ = lidar["vertical_fov_angle"].as<double>() * M_PI / 180.0;
  detection_probability_alpha_ = lidar["detection_probability_alpha"].as<double>();
  noise_std_dev_base_ = lidar["noise_std_dev_base"].as<double>();
  noise_scales_with_range_ = lidar["noise_scales_with_range"].as<bool>();
  noise_range_scaling_ = lidar["noise_range_scaling"].as<double>();
  mounting_pitch_ = lidar["mounting_pitch"].as<double>() * M_PI / 180.0;
}

std::vector<common_lib::structures::Cone> SimulatedPerception::perception_error(
    const std::vector<common_lib::structures::Cone>& cones,
    const common_lib::structures::Pose& vehicle_pose,
    const common_lib::structures::Velocities& vehicle_velocities) {
  std::vector<common_lib::structures::Cone> result;

  double yaw_cos = std::cos(vehicle_pose.orientation);
  double yaw_sin = std::sin(vehicle_pose.orientation);

  // Transform each cone
  for (const auto& cone : cones) {
    auto noisy_cone = cone;

    // Translate to vehicle-relative coordinates
    double dx = cone.position.x - vehicle_pose.position.x;
    double dy = cone.position.y - vehicle_pose.position.y;
    double dz = 0.0;
    
    // Rotate into LiDAR frame (accounting for yaw only)
    double x_vehicle = yaw_cos * dx + yaw_sin * dy;
    double y_vehicle = -yaw_sin * dx + yaw_cos * dy;
    double z_vehicle = dz - height_;

    // Convert to spherical coordinates
    double range_2d = std::sqrt(x_vehicle * x_vehicle + y_vehicle * y_vehicle);
    double azimuth = std::atan2(y_vehicle, x_vehicle);
    double elevation = std::atan2(z_vehicle, range_2d);
    double range_3d = std::sqrt(range_2d * range_2d + z_vehicle * z_vehicle);

    // Apply pitch in spherical space
    double elevation_with_pitch = elevation + mounting_pitch_ * std::cos(azimuth);

    // Convert back to Cartesian in LiDAR local frame
    double x_local = range_3d * std::cos(elevation_with_pitch) * std::cos(azimuth);
    double y_local = range_3d * std::cos(elevation_with_pitch) * std::sin(azimuth);
    double z_local = range_3d * std::sin(elevation_with_pitch);

    // Check field of view
    bool is_visible = true;

    double v_fov_half = vertical_fov_angle_ / 2.0;
    if (std::abs(elevation_with_pitch) > v_fov_half) {
      is_visible = false;
    }

    double h_fov_half = horizontal_fov_angle_ / 2.0;
    if (std::abs(azimuth) > h_fov_half) {
      is_visible = false;
    }

    if (is_visible && range_3d <= max_range_) {
      // Calculate noise standard deviation
      double sigma = noise_std_dev_base_;
      if (noise_scales_with_range_) {
        sigma += noise_range_scaling_ * range_3d;
      }

      // Apply Gaussian noise to coordinates
      double x_noisy = x_local + gaussian_noise(sigma);
      double y_noisy = y_local + gaussian_noise(sigma);

      // Update the cone's position with noisy values
      noisy_cone.position.x = x_noisy;
      noisy_cone.position.y = y_noisy;
    }

    result.push_back(noisy_cone);
  }

  return result;
}
