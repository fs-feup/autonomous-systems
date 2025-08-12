#include "fov_trimming/fov_trimming.hpp"

FovTrimming::FovTrimming(const std::shared_ptr<TrimmingParameters> params) : params_(params) {}

void FovTrimming::compute_rotation_constants(double max_range, double fov_trim_angle) {
  height_limit_ = params_->max_height - params_->lidar_height;
  squared_min_range_ = params_->min_range * params_->min_range;
  squared_max_range_ = params_->max_range * max_range;

  rot_rad_ = params_->lidar_rotation * M_PI / 180.0;
  cos_rot_ = std::cos(rot_rad_);
  sin_rot_ = std::sin(rot_rad_);

  pitch_rad_ = params_->lidar_pitch * M_PI / 180.0;
  cos_pitch_ = std::cos(pitch_rad_);
  sin_pitch_ = std::sin(pitch_rad_);

  fov_angle_rad_ = fov_trim_angle * M_PI / 180.0;
  min_angle_ = -fov_angle_rad_;
  max_angle_ = fov_angle_rad_;
}

void FovTrimming::process_point(pcl::PointXYZI& point, double& squared_distance,
                                double& angle) const {
  // Apply rotation
  double x_rot = point.x * cos_rot_ - point.y * sin_rot_;
  double y_rot = point.x * sin_rot_ + point.y * cos_rot_;

  // Apply pitch
  double x_pitch = x_rot * cos_pitch_ - point.z * sin_pitch_;
  double z_pitch = x_rot * sin_pitch_ + point.z * cos_pitch_;

  // Update the point coordinates in-place
  point.x = x_pitch;
  point.y = y_rot;
  point.z = z_pitch;

  squared_distance = point.x * point.x + point.y * point.y;
  angle = std::atan2(point.y, point.x);
}

bool FovTrimming::within_limits(pcl::PointXYZI& point) const {
  double squared_distance = 0, angle = 0;
  process_point(point, squared_distance, angle);

  bool within_height = point.z < height_limit_;
  bool within_range =
      (squared_distance > squared_min_range_) && (squared_distance <= squared_max_range_);
  bool within_fov = (angle >= min_angle_) && (angle <= max_angle_);

  return within_height && within_range && within_fov;
}

void FovTrimming::set_lidar_rotation(const double rotation) { params_->lidar_rotation = rotation; }

void FovTrimming::set_lidar_pitch(const double pitch) { params_->lidar_pitch = pitch; }
