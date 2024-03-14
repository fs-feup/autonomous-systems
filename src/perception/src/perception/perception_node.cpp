#include "perception/perception_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <cstdio>
#include <string>

#include "adapter/fsds.hpp"
#include "adapter/map.hpp"
#include "adapter/testlidar.hpp"
Perception::Perception(GroundRemoval* groundRemoval, Clustering* clustering,
                       ConeDifferentiation* coneDifferentiator, ConeValidator* coneValidator)
    : Node("perception"),
      groundRemoval(groundRemoval),
      clustering(clustering),
      coneDifferentiator(coneDifferentiator),
      coneValidator(coneValidator) {
  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);

  this->adapter = adapter_map[mode](this);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::savePCDFileASCII("raw_point_cloud.pcd", *pcl_cloud);

  groundRemoval->groundRemoval(pcl_cloud, ground_removed_cloud);

  pcl::io::savePCDFileASCII("ground_removed_cloud.pcd", *ground_removed_cloud);

  std::vector<Cluster> clusters;

  clustering->clustering(ground_removed_cloud, &clusters);

  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", clusters.size());

  int trueVals = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr clusterscloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cones(new pcl::PointCloud<pcl::PointXYZI>);
  for (int i = 0; i < clusters.size(); i++) {
    coneDifferentiator->coneDifferentiation(&clusters[i]);
    bool temp = coneValidator->coneValidator(&clusters[i]);
    if (temp) {
      trueVals++;
      cones->push_back(pcl::PointXYZI(clusters[i].getCentroid().x(), clusters[i].getCentroid().y(), clusters[i].getCentroid().z(), 1.0));
      
      }
    std::string color = clusters[i].getColor();
    RCLCPP_DEBUG(this->get_logger(), "Cone %d: %s", i, color.c_str());
    clusterscloud->push_back(pcl::PointXYZI(clusters[i].getCentroid().x(), clusters[i].getCentroid().y(), clusters[i].getCentroid().z(), 1.0));
  }

  pcl::io::savePCDFileASCII("clusters.pcd", *clusterscloud);

  pcl::io::savePCDFileASCII("cones.pcd", *cones);

  RCLCPP_INFO(this->get_logger(), "Total Clusters: %d Total Cones: %d", clusters.size(), trueVals);

  publishCones(&clusters);
}

void Perception::publishCones(std::vector<Cluster>* cones) {
  auto message = custom_interfaces::msg::ConeArray();
  for (int i = 0; i < cones->size(); i++) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = cones->at(i).getCentroid().x();
    position.y = cones->at(i).getCentroid().y();

    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = cones->at(i).getColor();
    message.cone_array.push_back(cone_message);
  }

  this->_cones_publisher->publish(message);
}
