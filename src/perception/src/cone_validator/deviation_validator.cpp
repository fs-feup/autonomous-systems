#include <cone_validator/deviation_validator.hpp>
#include <cmath>
#include <numeric>
#include <vector>

DeviationValidator::DeviationValidator(double min_xoy, 
        double max_xoy, double min_z, double max_z) : 
        min_xoy(min_xoy), max_xoy(max_xoy), min_z(min_z), max_z(max_z) {}

bool DeviationValidator::coneValidator(Cluster* cone_point_cloud, [[maybe_unused]]Plane& plane) const {
    // Vectors to store deviations in XOY and Z
    std::vector<double> deviations_xoy;
    std::vector<double> deviations_z;

    // Calculate the mean point in XOY and Z
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    int num_points = cone_point_cloud->get_point_cloud()->points.size();
    for (const auto& point : cone_point_cloud->get_point_cloud()->points) {
        mean_x += point.x;
        mean_y += point.y;
        mean_z += point.z;
    }
    mean_x /= num_points;
    mean_y /= num_points;
    mean_z /= num_points;

    // Calculate deviations from the mean
    for (const auto& point : cone_point_cloud->get_point_cloud()->points) {
        double deviation_xoy = std::sqrt(std::pow(point.x - mean_x, 2) + std::pow(point.y - mean_y, 2));
        deviations_xoy.push_back(deviation_xoy);
        deviations_z.push_back(std::abs(point.z - mean_z));
    }

    // Calculate the standard deviation of the deviations
    auto calc_std_dev = [](const std::vector<double>& deviations) {
        double sum = std::accumulate(deviations.begin(), deviations.end(), 0.0);
        double mean = sum / deviations.size();
        double sq_sum = std::inner_product(deviations.begin(), deviations.end(), deviations.begin(), 0.0);
        double variance = sq_sum / deviations.size() - mean * mean;
        return std::sqrt(variance);
    };

    double std_dev_xoy = calc_std_dev(deviations_xoy);
    double std_dev_z = calc_std_dev(deviations_z);

    // Validate against the thresholds
    return (std_dev_xoy >= min_xoy && std_dev_xoy <= max_xoy) && (std_dev_z >= min_z && std_dev_z <= max_z);
}
