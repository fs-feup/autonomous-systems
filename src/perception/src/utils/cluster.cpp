#include <utils/cluster.hpp>

Cluster::Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
    : _point_cloud_(point_cloud), _color_("undefined") {}

Eigen::Vector4f Cluster::getCentroid() {
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

std::string Cluster::getColor() { return _color_; }

void Cluster::setColor(const std::string& new_color) {
  if (new_color == "blue" || new_color == "yellow") this->_color_ = new_color;
}

// cppcheck-suppress unusedFunction
void Cluster::setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud) {
  _point_cloud_.reset();

  this->_point_cloud_ = new_point_cloud;

  _centroid_is_defined_ = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Cluster::getPointCloud() { return this->_point_cloud_; }

void Cluster::setConfidence(double newConfidence) { this->_confidence_ = newConfidence; }

double Cluster::getConfidence() { return this->_confidence_; }