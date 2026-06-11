#include "sensors/simulated_perception.hpp"

#include <cmath>
#include <algorithm>
#include <random>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <common_lib/config_load/config_load.hpp>
#include <common_lib/competition_logic/color.hpp>

SimulatedPerception::SimulatedPerception(const std::string& config_path) {

  std::string perception_cfg = common_lib::config_load::get_config_yaml_path(
        "invictasim", "invictasim/sensors", "perception");

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
  outlier_probability_ = lidar["outlier_probability"].as<double>();
  persistent_outlier_chance_ = lidar["persistent_outlier_chance"].as<double>();
}

std::vector<common_lib::structures::Cone> SimulatedPerception::perception_error(
    const std::vector<common_lib::structures::Cone>& cones,
    const common_lib::structures::Pose& vehicle_pose,
    const common_lib::structures::Velocities& vehicle_velocities) {
  std::vector<common_lib::structures::Cone> result;

  double yaw_cos = std::cos(vehicle_pose.orientation);
  double yaw_sin = std::sin(vehicle_pose.orientation);

  std::vector<common_lib::structures::Cone> cones_to_process = cones;
  cones_to_process.insert(
      cones_to_process.end(), persistent_outliers_.begin(), persistent_outliers_.end());

  // Generate random outliers in the field of view
  std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
  std::uniform_real_distribution<double> range_dist(0.0, max_range_);
  std::uniform_real_distribution<double> azimuth_dist(-horizontal_fov_angle_ / 2.0,
                                                      horizontal_fov_angle_ / 2.0);
  std::uniform_real_distribution<double> elevation_dist(-vertical_fov_angle_ / 2.0,
                                                        vertical_fov_angle_ / 2.0);

  // Generate outliers based on probability
  int num_outliers = 0;
  if (uniform_dist(generator_) < outlier_probability_) {
    num_outliers = 1;
  }

  for (int i = 0; i < num_outliers; ++i) {
    double random_range = range_dist(generator_);
    double random_azimuth = azimuth_dist(generator_);
    double random_elevation = elevation_dist(generator_);

    double x_local = random_range * std::cos(random_elevation) * std::cos(random_azimuth);
    double y_local = random_range * std::cos(random_elevation) * std::sin(random_azimuth);

    double dx_global = yaw_cos * x_local - yaw_sin * y_local;
    double dy_global = yaw_sin * x_local + yaw_cos * y_local;

    common_lib::structures::Cone outlier_cone;
    outlier_cone.position.x = vehicle_pose.position.x + dx_global;
    outlier_cone.position.y = vehicle_pose.position.y + dy_global;
    outlier_cone.color = common_lib::competition_logic::Color::UNKNOWN;
    outlier_cone.is_large = false;
    outlier_cone.certainty = 1.0;

    cones_to_process.push_back(outlier_cone);

    if (uniform_dist(generator_) < persistent_outlier_chance_) {
      persistent_outliers_.push_back(outlier_cone);
    }
  }

  // Transform each cone
  for (const auto& cone : cones_to_process) {
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
      auto noisy_cone = cone;
      double x_noisy = x_local + gaussian_noise(sigma);
      double y_noisy = y_local + gaussian_noise(sigma);

      noisy_cone.position.x = x_noisy;
      noisy_cone.position.y = y_noisy;
  
      result.push_back(noisy_cone);
    }
  }

  return result;
}
