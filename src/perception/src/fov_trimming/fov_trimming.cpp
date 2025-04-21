#include "fov_trimming/fov_trimming.hpp"

void FovTrimming::process_point(pcl::PointXYZI& point, const double rotation, const double pitch,
                                double& distance, double& angle) const {
  const double x_rot = point.x;
  const double y_rot = point.y;

  // Convert rotation angle to radians
  const double rot_rad = rotation * M_PI / 180.0;

  // Apply rotation transformation
  point.x = x_rot * std::cos(rot_rad) - y_rot * std::sin(rot_rad);
  point.y = x_rot * std::sin(rot_rad) + y_rot * std::cos(rot_rad);

  const double z_pitch = point.z;
  const double x_pitch = point.x;

  // Convert pitch angle to radians
  const double pitch_rad = pitch * M_PI / 180.0;

  // Apply pitch transformation
  point.x = x_pitch * std::cos(pitch_rad) - z_pitch * std::sin(pitch_rad);
  point.z = x_pitch * std::sin(pitch_rad) + z_pitch * std::cos(pitch_rad);

  // Calculate distance from LIDAR on the x0y plane
  distance = std::sqrt(point.x * point.x + point.y * point.y);

  // Calculate the angle of the point in the XY plane in degrees
  angle = std::atan2(point.y, point.x) * 180 / M_PI;
}

bool FovTrimming::within_limits(pcl::PointXYZI& point, const TrimmingParameters& params,
                                const double max_range, const double fov_trim_angle) const {
  double distance = 0, angle = 0;
  process_point(point, params.lidar_rotation, params.lidar_pitch, distance, angle);

  const bool within_height = point.z < (params.max_height - params.lidar_height);
  const bool within_range = (distance > params.min_range) && (distance <= max_range);
  const bool within_fov = (angle >= -fov_trim_angle) && (angle <= fov_trim_angle);

  return within_height && within_range && within_fov;
}

void FovTrimming::set_lidar_rotation(const double rotation) { params_.lidar_rotation = rotation; }

void FovTrimming::set_lidar_pitch(const double pitch) { params_.lidar_pitch = pitch; }
