#include <utils/cluster.hpp>

Cluster::Cluster(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud,
                 const std::vector<int>& point_indices)
    : _point_cloud_(point_cloud), _point_indices_(point_indices) {}

Eigen::Vector4f Cluster::get_centroid() {
  if (_centroid_is_defined_) return _centroid_;

  this->_centroid_ =
      Cluster::centroid_calculator.calculate_center(this->_point_cloud_, this->_point_indices_);
  this->_centroid_is_defined_ = true;
  return this->_centroid_;
}

Eigen::Vector4f Cluster::get_center(Plane& plane) {
  if (_center_is_defined_) return this->_center_;

  this->_center_ = Cluster::center_calculator.calculate_center(this->_point_cloud_,
                                                               this->_point_indices_, plane);
  this->_center_is_defined_ = true;

  return this->_center_;
}

std::string Cluster::get_color() { return _color_; }

void Cluster::set_color(const std::string& new_color) {
  if (new_color == "blue" || new_color == "yellow") this->_color_ = new_color;
}

// cppcheck-suppress unusedFunction
void Cluster::set_point_indices(const std::vector<int>& new_point_indices) {
  this->_point_indices_ = new_point_indices;
  this->_center_is_defined_ = false;
  this->_centroid_is_defined_ = false;
}

const std::vector<int>& Cluster::get_point_indices() { return this->_point_indices_; }

const sensor_msgs::msg::PointCloud2::SharedPtr& Cluster::get_point_cloud() {
  return this->_point_cloud_;
}

void Cluster::set_confidence(double new_confidence) { this->_confidence_ = new_confidence; }

double Cluster::get_confidence() { return this->_confidence_; }

void Cluster::set_z_score(double mean_x, double std_dev_x, double mean_y, double std_dev_y) {
  _z_score_x_ = std::abs(this->get_centroid().x() - mean_x) / std_dev_x;
  _z_score_y_ = std::abs(this->get_centroid().y() - mean_y) / std_dev_y;
  if (std_dev_x == 0) _z_score_x_ = 1;
  if (std_dev_y == 0) _z_score_y_ = 1;
}

double Cluster::get_z_score_x() const { return _z_score_x_; }

double Cluster::get_z_score_y() const { return _z_score_y_; }

std::tuple<double, double, double, double> Cluster::calculate_mean_and_std_dev(
    std::vector<Cluster>& clusters) {
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (auto& point : clusters) {
    sum_x += point.get_centroid().x();
    sum_y += point.get_centroid().y();
  }

  double mean_x = sum_x / static_cast<double>(clusters.size());
  double mean_y = sum_y / static_cast<double>(clusters.size());

  double sum_squared_diff_x = 0.0;
  double sum_squared_diff_y = 0.0;
  for (auto& point : clusters) {
    sum_squared_diff_x += pow(point.get_centroid().x() - mean_x, 2);
    sum_squared_diff_y += pow(point.get_centroid().y() - mean_y, 2);
  }

  double stddev_x = sqrt(sum_squared_diff_x / static_cast<double>(clusters.size()));
  double stddev_y = sqrt(sum_squared_diff_y / static_cast<double>(clusters.size()));

  return std::make_tuple(mean_x, mean_y, stddev_x, stddev_y);
}

void Cluster::set_z_scores(std::vector<Cluster>& clusters) {
  auto [mean_x, mean_y, stddev_x, stddev_y] = calculate_mean_and_std_dev(clusters);
  for (auto& cluster : clusters) {
    cluster.set_z_score(mean_x, stddev_x, mean_y, stddev_y);
  }
}

bool Cluster::get_is_large() { return _is_large_; }

void Cluster::set_is_large() { _is_large_ = true; }