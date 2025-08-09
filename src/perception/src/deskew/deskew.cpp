#include "deskew/deskew.hpp"

void Deskew::deskew_point_cloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    const common_lib::structures::Velocities& vehicle_velocity) {
    if (input_cloud->empty()) {
        return;
    }

    const Eigen::Vector3d linear_velocity(
        vehicle_velocity.velocity_x,
        vehicle_velocity.velocity_y,
        0.0);

    const double angular_velocity_z = static_cast<double>(vehicle_velocity.rotational_velocity);
    const double one_over_two_pi = 1.0 / (2.0 * M_PI);

    const pcl::PointXYZI& reference_point = input_cloud->points.front();
    double reference_azimuth = std::atan2(reference_point.x, reference_point.y);
    if (reference_azimuth < 0.0) {
        reference_azimuth += 2.0 * M_PI;
    }

    for (pcl::PointXYZI& point_in_cloud : *input_cloud) {
        // Step 1: Compute azimuth of this point (0° = +Y, CW positive)
        double azimuth = std::atan2(point_in_cloud.x, point_in_cloud.y);
        if (azimuth < 0.0) {
            azimuth += 2.0 * M_PI;
        }
        // Step 2: Compute time offset based on angular position
        double delta_angle = azimuth - reference_azimuth;
        if (delta_angle < 0.0) {
            delta_angle += 2.0 * M_PI;
        }

        double time_offset = delta_angle * one_over_two_pi * 0.05;
        // Step 3: Compute new position by transforming the point forward in time
        Eigen::Vector3d point_coordinates(point_in_cloud.x, point_in_cloud.y, point_in_cloud.z);
        // Add the motion (translation)
        point_coordinates += linear_velocity * time_offset;

        // Add the rotation (about Z axis)
        if (angular_velocity_z != 0.0) {
            double theta = angular_velocity_z * time_offset;
            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);

            double rotated_x = cos_theta * point_coordinates.x() - sin_theta * point_coordinates.y();
            double rotated_y = sin_theta * point_coordinates.x() + cos_theta * point_coordinates.y();

            point_in_cloud.x = rotated_x;
            point_in_cloud.y = rotated_y;
        } else {
            point_in_cloud.x = point_coordinates.x();
            point_in_cloud.y = point_coordinates.y();
        }

        point_in_cloud.z = point_coordinates.z(); // Z unchanged by yaw rotation
    }
}


std::vector<std::pair<double,double>> Deskew::deskew_clusters(std::vector<Cluster>& clusters, const common_lib::structures::Velocities& vel) {
    double x, y;
    Eigen::Vector4f centroid;
    double angle, elapsed_time;
    double scan_duration = 0.1;

    RCLCPP_INFO(rclcpp::get_logger("skew"), "velocities: %f, %f, %f", vel.velocity_x, vel.velocity_y, vel.rotational_velocity);

    std::vector<std::pair<double,double>> result;

    for (auto& cluster : clusters) {
        centroid = cluster.get_centroid();
        x = centroid.x();
        y = centroid.y();
        angle = atan2(y, x);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
        elapsed_time = scan_duration * angle / (2 * M_PI);
        x -= elapsed_time * vel.velocity_x;
        y -= elapsed_time * vel.velocity_y;
        result.emplace_back(std::make_pair(x, y));
    }
    return result;
}