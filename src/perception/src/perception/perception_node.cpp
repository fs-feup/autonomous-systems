#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>
#include <string>
#include <vector>

#include "adapter/fsds.hpp"
#include "adapter/map.hpp"
#include "adapter/testlidar.hpp"

Perception::Perception(GroundRemoval* groundRemoval, Clustering* clustering,
                       ConeDifferentiation* coneDifferentiator,
                       const std::vector<ConeValidator*>& coneValidators)
    : Node("perception"),
      groundRemoval(groundRemoval),
      clustering(clustering),
      coneDifferentiator(coneDifferentiator),
      coneValidators(coneValidators) {

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);

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

  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", clusters.size());

  for (long unsigned int i = 0; i < filtered_clusters.size(); i++) {
    coneDifferentiator->coneDifferentiation(&filtered_clusters[i]);
    std::string color = filtered_clusters[i].getColor();
    filtered_clusters[i].setColor(color);
    RCLCPP_DEBUG(this->get_logger(), "Cone %d: %s", i, color.c_str());
  }

  publishCones(&filtered_clusters);
}

void Perception::publishCones(std::vector<Cluster>* cones) {
  auto message = custom_interfaces::msg::ConeArray();
  for (int i = 0; i < static_cast<int>(cones->size()); i++) {
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
