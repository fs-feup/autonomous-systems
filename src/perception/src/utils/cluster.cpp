#include <utils/cluster.hpp>

Cluster::Cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud) : point_cloud(point_cloud) {
  color = "undefined";
  centroidIsDefined = false;
}

Eigen::Vector4f Cluster::getCentroid() {
  if (centroidIsDefined) return this->centroid;

  Eigen::Vector4f tempCentroid;
  pcl::compute3DCentroid(*point_cloud, centroid);
  this->centroidIsDefined = true;

  return centroid;
}

std::string Cluster::getColor() { return color; }

void Cluster::setColor(std::string new_color) {
  if (new_color == "blue" || new_color == "yellow") this->color = new_color;
}

void Cluster::setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud) {
  point_cloud.reset();

  this->point_cloud = new_point_cloud;

  centroidIsDefined = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Cluster::getPointCloud() { return this->point_cloud; }