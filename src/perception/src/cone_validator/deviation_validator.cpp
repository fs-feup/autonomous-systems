#include <cone_validator/deviation_validator.hpp>

DeviationValidator::DeviationValidator(double min_xoy, double max_xoy, double min_z, double max_z)
    : _min_xoy_(min_xoy), _max_xoy_(max_xoy), _min_z_(min_z), _max_z_(max_z) {}

std::vector<double> DeviationValidator::coneValidator(Cluster* cone_cluster,
                                                      [[maybe_unused]] Plane& plane) const {
  // Vectors to store deviations in XOY and Z
  std::vector<double> deviations_xoy;
  std::vector<double> deviations_z;

  const auto& cloud_data = cone_cluster->get_point_cloud()->data;
  const auto& indices = cone_cluster->get_point_indices();

  // Calculate the mean point in XOY and Z
  double mean_x = 0.0;
  double mean_y = 0.0;
  double mean_z = 0.0;
  for (size_t idx : indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);

    mean_x += x;
    mean_y += y;
    mean_z += z;
  }
  mean_x /= static_cast<double>(indices.size());
  mean_y /= static_cast<double>(indices.size());
  mean_z /= static_cast<double>(indices.size());

  // Calculate deviations from the mean
  for (size_t idx : indices) {
    float x = *reinterpret_cast<const float*>(&cloud_data[PointX(idx)]);
    float y = *reinterpret_cast<const float*>(&cloud_data[PointY(idx)]);
    float z = *reinterpret_cast<const float*>(&cloud_data[PointZ(idx)]);

    double deviation_xoy = std::sqrt(std::pow(x - mean_x, 2) + std::pow(y - mean_y, 2));
    deviations_xoy.push_back(deviation_xoy);
    deviations_z.push_back(std::abs(z - mean_z));
  }

  // Calculate the standard deviation of the deviations
  auto calc_std_dev = [](const std::vector<double>& deviations) {
    double sum = std::accumulate(deviations.begin(), deviations.end(), 0.0);
    double mean = sum / static_cast<double>(deviations.size());
    double sq_sum =
        std::inner_product(deviations.begin(), deviations.end(), deviations.begin(), 0.0);
    double variance = sq_sum / static_cast<double>(deviations.size()) - mean * mean;
    return std::sqrt(variance);
  };

  double std_dev_xoy = calc_std_dev(deviations_xoy);
  double std_dev_z = calc_std_dev(deviations_z);

  // index 0 = if not in xoy interval, ratio between the std deviation of the cluster and the
  // maximum or minimum deviation, whichever is closest to.
  // index 1 = if not z in interval, ratio between the std deviation of the cluster and the maximum
  // or minimum deviation, whichever is the closest to z
  return {std::min({_min_xoy_ > 0 ? std_dev_xoy / _min_xoy_ : 1.0, _max_xoy_ / std_dev_xoy, 1.0}),
          std::min({_min_z_ > 0 ? std_dev_z / _min_z_ : 1.0, _max_z_ / std_dev_z, 1.0})};
}
