#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>
#include <string>
#include <vector>

#include "adapter_perception/fsds.hpp"
#include "adapter_perception/vehicle.hpp"
#include "std_msgs/msg/header.hpp"

std_msgs::msg::Header header;

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _ground_removal_(params.ground_removal_),
      _clustering_(params.clustering_),
      _cone_differentiator_(params.cone_differentiator_),
      _cone_validators_(params.cone_validators_),
      _cone_evaluator_(params.distance_predict_) {
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  header = (*msg).header;

  pcl::fromROSMsg(*msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  _ground_removal_->ground_removal(pcl_cloud, ground_removed_cloud, _ground_plane_);

  std::vector<Cluster> clusters;

  _clustering_->clustering(ground_removed_cloud, &clusters);

  std::vector<Cluster> filtered_clusters;

  for (auto cluster : clusters) {
    if (std::all_of(_cone_validators_.begin(), _cone_validators_.end(), [&](const auto& validator) {
          return validator->coneValidator(&cluster, _ground_plane_);
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
    _cone_differentiator_->coneDifferentiation(&filtered_clusters[i]);
    std::string color = filtered_clusters[i].get_color();
    filtered_clusters[i].set_color(color);
    RCLCPP_DEBUG(this->get_logger(), "Cone %d: %s", i, color.c_str());
  }

  publishCones(&clusters);
}

void Perception::publishCones(std::vector<Cluster>* cones) {
  auto message = custom_interfaces::msg::ConeArray();
  message.header = header;
  for (int i = 0; i < static_cast<int>(cones->size()); i++) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = cones->at(i).get_centroid().x();
    position.y = cones->at(i).get_centroid().y();

    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = cones->at(i).get_color();
    cone_message.confidence = cones->at(i).get_confidence();
    message.cone_array.push_back(cone_message);
  }

  this->_cones_publisher->publish(message);
}
