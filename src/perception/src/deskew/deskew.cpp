#include "deskew/deskew.hpp"

void Deskew::deskew_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud,
                                const common_lib::structures::Velocities& vehicle_velocity) const {
  const size_t num_points = input_cloud->width * input_cloud->height;
  if (num_points == 0) {
    return;
  }

  constexpr double scan_duration = 0.1;
  constexpr double one_over_two_pi = 1.0 / (2.0 * M_PI);

  // Vehicle velocities
  const Eigen::Vector3d linear_velocity(vehicle_velocity.velocity_x, vehicle_velocity.velocity_y,
                                        0.0);

  const double angular_velocity_z = static_cast<double>(vehicle_velocity.rotational_velocity);

  auto& cloud = input_cloud->data;

  // Reference point (last in scan)
  int ref_index = static_cast<int>(num_points - 1);
  double ref_x = *reinterpret_cast<const float*>(&cloud[PointX(ref_index)]);
  double ref_y = *reinterpret_cast<const float*>(&cloud[PointY(ref_index)]);
  double reference_azimuth = std::atan2(ref_x, ref_y);
  if (reference_azimuth < 0.0) {
    reference_azimuth += 2.0 * M_PI;
  }

  // Deskew each point
  for (size_t i = 0; i < num_points; ++i) {
    float x = *reinterpret_cast<const float*>(&cloud[PointX(i)]);
    float y = *reinterpret_cast<const float*>(&cloud[PointY(i)]);
    float z = *reinterpret_cast<const float*>(&cloud[PointZ(i)]);

    double azimuth = std::atan2(x, y);
    if (azimuth < 0.0) {
      azimuth += 2.0 * M_PI;
    }

    double delta_angle = reference_azimuth - azimuth;
    if (delta_angle < 0.0) {
      delta_angle += 2.0 * M_PI;
    }

    double time_offset = delta_angle * one_over_two_pi * scan_duration;

    Eigen::Vector3d original_point(x, y, z);
    Eigen::Vector3d deskewed_point = original_point - linear_velocity * time_offset;

    // Reverse rotation around Z
    if (angular_velocity_z != 0.0) {
      double theta = -angular_velocity_z * time_offset;
      double cos_theta = std::cos(theta);
      double sin_theta = std::sin(theta);

      double x_rot = cos_theta * deskewed_point.x() - sin_theta * deskewed_point.y();
      double y_rot = sin_theta * deskewed_point.x() + cos_theta * deskewed_point.y();

      deskewed_point.x() = x_rot;
      deskewed_point.y() = y_rot;
    }

    // Write all 3 floats in one memcpy
    float deskewed_xyz[3] = {static_cast<float>(deskewed_point.x()),
                             static_cast<float>(deskewed_point.y()),
                             static_cast<float>(deskewed_point.z())};

    std::memcpy(&cloud[PointX(i)], deskewed_xyz, sizeof(deskewed_xyz));
  }
}
