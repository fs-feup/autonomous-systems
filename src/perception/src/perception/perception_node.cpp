#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>
#include <string>
#include <vector>

#include "adapter_perception/fsds.hpp"
#include "adapter_perception/map.hpp"
#include "adapter_perception/vehicle.hpp"
#include "std_msgs/msg/header.hpp"

std_msgs::msg::Header header;

Perception::Perception(std::shared_ptr<GroundRemoval> ground_removal, std::shared_ptr<Clustering> clustering,
             std::shared_ptr<ConeDifferentiation> cone_differentiator,
             const std::vector<std::shared_ptr<ConeValidator>>& cone_validators, 
             std::shared_ptr<ConeEvaluator> cone_evaluator, std::string mode)
    : Node("perception"),
      _ground_removal_(ground_removal),
      _clustering_(clustering),
      _cone_differentiator_(cone_differentiator),
      _cone_validators_(cone_validators),
      _cone_evaluator_(cone_evaluator),
      _mode_(mode) {
  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

  this->_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mockingjay", 10);

  this->_adapter_ = std::shared_ptr<Adapter>(adapter_map[mode](this));
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
  int i = 0;
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
    if (i == 0)
      pcl::io::savePCDFileASCII("../../cluster.pcd", *filtered_clusters[i].get_point_cloud());
  }

  publishCones(&clusters);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>("filtered_output_modified.pcd", *pcl_cloud2) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file");
  }

  RCLCPP_INFO(this->get_logger(), "Size: %d", pcl_cloud2->size());

  sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*pcl_cloud2, *ros_cloud);

  this->_pc_publisher->publish(*ros_cloud);
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
