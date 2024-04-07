#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>
#include <string>

#include "adapter/fsds.hpp"
#include "adapter/map.hpp"
#include "adapter/testlidar.hpp"
#include <vector>

Perception::Perception(GroundRemoval* groundRemoval, Clustering* clustering,
                       ConeDifferentiation* coneDifferentiator,
                       const std::vector<ConeValidator*>& coneValidators)
    : Node("perception"),
      groundRemoval(groundRemoval),
      clustering(clustering),
      coneDifferentiator(coneDifferentiator),
      coneValidators(coneValidators) {
  this->_cones_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("perception/vis/cone_coordinates", 10);

  this->adapter = adapter_map[mode](this);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  groundRemoval->groundRemoval(pcl_cloud, ground_removed_cloud, groundPlane);

  std::vector<Cluster> clusters;

  clustering->clustering(ground_removed_cloud, &clusters);

  std::vector<Cluster> filtered_clusters;

  for (auto cluster : clusters) {
      if (std::all_of(coneValidators.begin(), coneValidators.end(), [&](const auto& validator) {
          return validator->coneValidator(&cluster, groundPlane);
      })) {
          filtered_clusters.push_back(cluster);
      }
  }

  filtered_clusters = clusters;


  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", clusters.size());


  for (int i = 0; i < filtered_clusters.size(); i++) {
    coneDifferentiator->coneDifferentiation(&filtered_clusters[i]);
    std::string color = filtered_clusters[i].getColor();
    RCLCPP_DEBUG(this->get_logger(), "Cone %d: %s", i, color.c_str());
  }


  publishCones(&filtered_clusters);
}


void Perception::publishCones(std::vector<Cluster>* clusters) {
    visualization_msgs::msg::MarkerArray marker_array;
    int count = 0;

    if (clusters->empty()) {
      RCLCPP_WARN(this->get_logger(), "No clusters to publish.");
      return;
    }

    marker_array.markers = {};

    for (int i = 0; i < clusters->size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "Cluster Centroids";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = clusters->at(i).getCentroid().x();
        marker.pose.position.y = clusters->at(i).getCentroid().y();
        marker.pose.position.z = clusters->at(i).getCentroid().z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.4;
        marker.scale.z = 0.2;
        marker.color.r = 0.55;
        marker.color.g = 0.1;
        marker.color.b = 0.69;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
        count++;
    }

    this->_cones_publisher->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "%d cones published", marker_array.markers.size());
}

