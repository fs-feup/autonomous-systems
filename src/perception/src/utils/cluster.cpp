#include <utils/cluster.hpp>

Cluster::Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
    : _point_cloud_(point_cloud) {}

Eigen::Vector4f Cluster::get_centroid() {
  if (_centroid_is_defined_) return this->_centroid_;

  pcl::compute3DCentroid(*_point_cloud_, _centroid_);
  this->_centroid_is_defined_ = true;

  return _centroid_;
}

Eigen::Vector4f Cluster::get_center(Plane& plane) {
  if (_center_is_defined_) return this->_center_;

  this->_center_ = Cluster::center_calculator.calculate_center(this->_point_cloud_, plane);
  this->_center_is_defined_ = true;

  return this->_center_;
}

std::string Cluster::get_color() { return _color_; }

void Cluster::set_color(const std::string& new_color) {
  if (new_color == "blue" || new_color == "yellow") this->_color_ = new_color;
}

// cppcheck-suppress unusedFunction
void Cluster::set_point_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud) {
  _point_cloud_.reset();

  this->_point_cloud_ = new_point_cloud;

  _centroid_is_defined_ = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Cluster::get_point_cloud() { return this->_point_cloud_; }

void Cluster::set_confidence(double new_confidence) { this->_confidence_ = new_confidence; }

double Cluster::get_confidence() { return this->_confidence_; }