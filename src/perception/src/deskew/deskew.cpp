#include "deskew/deskew.hpp"

void Deskew::deskew_point_cloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    const common_lib::structures::Velocities& vehicle_velocity) {
    
    if (input_cloud->empty()) {
        return;
    }

    constexpr double scan_duration = 0.1; 
    constexpr double one_over_two_pi = 1.0 / (2.0 * M_PI);


    // Vehicle velocities
    const Eigen::Vector3d linear_velocity(
        vehicle_velocity.velocity_x,
        vehicle_velocity.velocity_y,
        0.0);

    const double angular_velocity_z = static_cast<double>(vehicle_velocity.rotational_velocity);


    // Get azimuth of the last point (i.e., end of scan)
    const pcl::PointXYZI& reference_point = input_cloud->points.back();
    double reference_azimuth = std::atan2(reference_point.x, reference_point.y);
    if (reference_azimuth < 0.0) {
        reference_azimuth += 2.0 * M_PI;
    }

    for (pcl::PointXYZI& point : *input_cloud) {
        // Step 1: Get azimuth of this point (0° = +Y, CW positive)
        double azimuth = std::atan2(point.x, point.y);
        if (azimuth < 0.0) {
            azimuth += 2.0 * M_PI;
        }

        // Step 2: Compute time difference from this point to end of scan
        double delta_angle = reference_azimuth - azimuth;
        if (delta_angle < 0.0) {
            delta_angle += 2.0 * M_PI;
        }

        // Time offset (in seconds) from this point to end of scan
        double time_offset = delta_angle * one_over_two_pi * scan_duration;

        // Step 3: Undo the motion during that time
        Eigen::Vector3d original_point(point.x, point.y, point.z);

        // Reverse translation
        Eigen::Vector3d deskewed_point = original_point - linear_velocity * time_offset;
        // Reverse rotation (around Z axis)
        if (angular_velocity_z != 0.0) {
            double theta = -angular_velocity_z * time_offset; // Note the negative sign
            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            double x_rot = cos_theta * deskewed_point.x() - sin_theta * deskewed_point.y();
            double y_rot = sin_theta * deskewed_point.x() + cos_theta * deskewed_point.y();

            point.x = x_rot;
            point.y = y_rot;
        } else {
            point.x = deskewed_point.x();
            point.y = deskewed_point.y();
        }

        point.z = deskewed_point.z(); // Z is unaffected by yaw rotation
    }
}