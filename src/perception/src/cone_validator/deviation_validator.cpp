#include <cone_validator/deviation_validator.hpp>
#include <cmath>
#include <numeric>
#include <vector>
#include "rclcpp/rclcpp.hpp"

DeviationValidator::DeviationValidator(double min_xoy, 
        double max_xoy, double min_z, double max_z) : 
        _min_xoy_(min_xoy), _max_xoy_(max_xoy), _min_z_(min_z), _max_z_(max_z) {}

bool DeviationValidator::coneValidator(Cluster* cone_point_cloud, [[maybe_unused]]Plane& plane) const {
    // Vectors to store deviations in XOY and Z
    std::vector<double> deviations_xoy;
    std::vector<double> deviations_z;

    // Calculate the mean point in XOY and Z
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_z = 0.0;
    unsigned long num_points = cone_point_cloud->get_point_cloud()->points.size();
    for (const auto& point : cone_point_cloud->get_point_cloud()->points) {
        mean_x += point.x;
        mean_y += point.y;
        mean_z += point.z;
    }
    mean_x /= static_cast<double>(num_points);
    mean_y /= static_cast<double>(num_points);
    mean_z /= static_cast<double>(num_points);

    // Calculate deviations from the mean
    for (const auto& point : cone_point_cloud->get_point_cloud()->points) {
        double deviation_xoy = std::sqrt(std::pow(point.x - mean_x, 2) + std::pow(point.y - mean_y, 2));
        deviations_xoy.push_back(deviation_xoy);
        deviations_z.push_back(std::abs(point.z - mean_z));
    }

    // Calculate the standard deviation of the deviations
    auto calc_std_dev = [](const std::vector<double>& deviations) {
        double sum = std::accumulate(deviations.begin(), deviations.end(), 0.0);
        double mean = sum / static_cast<double>(deviations.size());
        double sq_sum = std::inner_product(deviations.begin(), deviations.end(), deviations.begin(), 0.0);
        double variance = sq_sum / static_cast<double>(deviations.size()) - mean * mean;
        return std::sqrt(variance);
    };

    double std_dev_xoy = calc_std_dev(deviations_xoy);
    double std_dev_z = calc_std_dev(deviations_z);

    // Validate against the thresholds
    if ((std_dev_xoy >= _min_xoy_ && std_dev_xoy <= _max_xoy_) && (std_dev_z >= _min_z_ && std_dev_z <= _max_z_)){
         RCLCPP_DEBUG(rclcpp::get_logger("perception-1"), "Valid! - xOy: %f - z: %f", std_dev_xoy, std_dev_z);
         return true;
    }
    else{
        RCLCPP_DEBUG(rclcpp::get_logger("perception-1"), "Invalid! - xOy: %f - z: %f", std_dev_xoy, std_dev_z);
        return false;
    }

}
