#include "fov_trimming/fov_trimming.hpp"

FovTrimming::FovTrimming(const std::shared_ptr<TrimmingParameters> params) : params_(params) {}

void FovTrimming::compute_rotation_constants(double max_range, double fov_trim_angle) {
  height_limit_ = params_->max_height - params_->lidar_height;
  squared_min_range_ = params_->min_range * params_->min_range;
  squared_max_range_ = max_range * max_range;

  rot_rad_ = params_->lidar_rotation * M_PI / 180.0;
  cos_rot_ = std::cos(rot_rad_);
  sin_rot_ = std::sin(rot_rad_);

  pitch_rad_ = params_->lidar_pitch * M_PI / 180.0;
  cos_pitch_ = std::cos(pitch_rad_);
  sin_pitch_ = std::sin(pitch_rad_);

  roll_rad_ = params_->lidar_roll * M_PI / 180.0;
  cos_roll_ = std::cos(roll_rad_);
  sin_roll_ = std::sin(roll_rad_);

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

  // Apply roll
  double y_roll = y_rot * cos_roll_ - z_pitch * sin_roll_;
  double z_roll = y_rot * sin_roll_ + z_pitch * cos_roll_;

  // Update the point coordinates in-place
  point.x = x_pitch;
  point.y = y_roll;
  point.z = z_roll;

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
