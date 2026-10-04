#include "sensors/simulated_perception.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "common_lib/competition_logic/color.hpp"
#include "config/yaml_helpers.hpp"

SimulatedPerception::SimulatedPerception(const std::string& config_path) {
  using invictasim::config::load_optional;

  const YAML::Node lidar = YAML::LoadFile(config_path)["lidar_sensor"];

  double horizontal_fov_degrees = 120.0;
  double vertical_fov_degrees = 20.0;
  double mounting_pitch_degrees = 0.0;

  load_optional(lidar, "height", height_);
  load_optional(lidar, "cone_height", cone_height_);
  load_optional(lidar, "max_range", max_range_);
  load_optional(lidar, "horizontal_fov_angle", horizontal_fov_degrees);
  load_optional(lidar, "vertical_fov_angle", vertical_fov_degrees);
  load_optional(lidar, "mounting_pitch", mounting_pitch_degrees);
  load_optional(lidar, "detection_probability_alpha", detection_probability_alpha_);
  load_optional(lidar, "noise_std_dev_base", noise_std_dev_base_);
  load_optional(lidar, "noise_scales_with_range", noise_scales_with_range_);
  load_optional(lidar, "noise_range_scaling", noise_range_scaling_);
  load_optional(lidar, "outlier_rate_hz", outlier_rate_hz_);
  load_optional(lidar, "persistent_outlier_chance", persistent_outlier_chance_);

  horizontal_fov_ = horizontal_fov_degrees * M_PI / 180.0;
  vertical_fov_ = vertical_fov_degrees * M_PI / 180.0;
  mounting_pitch_ = mounting_pitch_degrees * M_PI / 180.0;
}

void SimulatedPerception::reset() { persistent_outliers_.clear(); }

bool SimulatedPerception::is_in_fov(double x_vehicle, double y_vehicle) const {
  const double range_2d = std::hypot(x_vehicle, y_vehicle);
  if (range_2d <= 0.0) {
    return false;
  }

  if (std::abs(std::atan2(y_vehicle, x_vehicle)) > horizontal_fov_ / 2.0) {
    return false;
  }

  // Elevation of the cone's centroid as seen from the sensor, expressed in the sensor's own frame
  // so a nose-down mounting pitch brings nearer cones into view rather than shifting where they
  // are reported.
  const double elevation = std::atan2(cone_height_ - height_, range_2d);
  return std::abs(elevation + mounting_pitch_) <= vertical_fov_ / 2.0;
}

void SimulatedPerception::spawn_outliers(
    const common_lib::structures::Pose& vehicle_pose, double dt,
    std::vector<common_lib::structures::Cone>& cones_to_process) {
  if (!(outlier_rate_hz_ > 0.0) || !(dt > 0.0)) {
    return;
  }

  // Rate per second rather than per call, so the result does not depend on the simulation step.
  if (random_uniform() >= outlier_rate_hz_ * dt) {
    return;
  }

  const double yaw_cos = std::cos(vehicle_pose.orientation);
  const double yaw_sin = std::sin(vehicle_pose.orientation);
  const double range = random_uniform() * max_range_;
  const double azimuth = (random_uniform() - 0.5) * horizontal_fov_;

  const double x_local = range * std::cos(azimuth);
  const double y_local = range * std::sin(azimuth);

  common_lib::structures::Cone outlier;
  outlier.position.x = vehicle_pose.position.x + yaw_cos * x_local - yaw_sin * y_local;
  outlier.position.y = vehicle_pose.position.y + yaw_sin * x_local + yaw_cos * y_local;
  outlier.color = common_lib::competition_logic::Color::UNKNOWN;
  outlier.is_large = false;
  outlier.certainty = 1.0;

  cones_to_process.push_back(outlier);

  if (random_uniform() < persistent_outlier_chance_ &&
      persistent_outliers_.size() < kMaxPersistentOutliers) {
    persistent_outliers_.push_back(outlier);
  }
}

std::vector<common_lib::structures::Cone> SimulatedPerception::perception_error(
    const std::vector<common_lib::structures::Cone>& cones,
    const common_lib::structures::Pose& vehicle_pose, double dt) {
  const double yaw_cos = std::cos(vehicle_pose.orientation);
  const double yaw_sin = std::sin(vehicle_pose.orientation);

  std::vector<common_lib::structures::Cone> cones_to_process = cones;
  cones_to_process.insert(cones_to_process.end(), persistent_outliers_.begin(),
                          persistent_outliers_.end());
  spawn_outliers(vehicle_pose, dt, cones_to_process);

  std::vector<common_lib::structures::Cone> result;
  result.reserve(cones_to_process.size());

  for (const auto& cone : cones_to_process) {
    // Into the vehicle frame: x forward, y left.
    const double dx = cone.position.x - vehicle_pose.position.x;
    const double dy = cone.position.y - vehicle_pose.position.y;
    const double x_vehicle = yaw_cos * dx + yaw_sin * dy;
    const double y_vehicle = -yaw_sin * dx + yaw_cos * dy;

    if (!is_in_fov(x_vehicle, y_vehicle)) {
      continue;
    }

    const double range = std::hypot(x_vehicle, y_vehicle);

    // Detection probability falls off through 50% at max_range_.
    const double exponent =
        std::clamp(detection_probability_alpha_ * (range - max_range_), -100.0, 100.0);
    if (random_uniform() >= 1.0 / (1.0 + std::exp(exponent))) {
      continue;
    }

    double sigma = noise_std_dev_base_;
    if (noise_scales_with_range_) {
      sigma += noise_range_scaling_ * range;
    }

    common_lib::structures::Cone detected = cone;
    detected.position.x = x_vehicle + gaussian_noise(sigma);
    detected.position.y = y_vehicle + gaussian_noise(sigma);
    result.push_back(detected);
  }

  return result;
}
