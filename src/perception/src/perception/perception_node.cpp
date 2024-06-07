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
#include "common_lib/communication/marker.hpp"
#include "std_msgs/msg/header.hpp"

std_msgs::msg::Header header;

Perception::Perception(std::shared_ptr<GroundRemoval> ground_removal,
                       std::shared_ptr<Clustering> clustering,
                       std::shared_ptr<ConeDifferentiation> cone_differentiator,
                       const std::vector<std::shared_ptr<ConeValidator>>& cone_validators,
                       std::shared_ptr<ConeEvaluator> cone_evaluator, std::string mode,
                       std::shared_ptr<ICP> icp)
    : Node("perception"),
      _ground_removal_(ground_removal),
      _clustering_(clustering),
      _cone_differentiator_(cone_differentiator),
      _cone_validators_(cone_validators),
      _cone_evaluator_(cone_evaluator),
      _mode_(mode),
      _icp_(icp) {
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

  this->cone_marker_array =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/debug/cones", 10);
  this->_adapter_ = std::shared_ptr<Adapter>(adapter_map[mode](this));
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  header = (*msg).header;

  pcl::fromROSMsg(*msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  _ground_removal_->ground_removal(pcl_cloud, ground_removed_cloud, _ground_plane_);

  pcl::io::savePCDFileASCII("test_pcd11.pcd", *ground_removed_cloud);

  std::vector<Cluster> clusters;

  _clustering_->clustering(ground_removed_cloud, &clusters);

  pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_pc(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto cluster : clusters) {
    clusters_pc->points.push_back({cluster.get_centroid()[0], cluster.get_centroid()[1],
                                   cluster.get_centroid()[2], cluster.get_centroid()[3]});
  }

  clusters_pc->width = 1;
  clusters_pc->height = clusters.size();

  pcl::io::savePCDFileASCII("test_pcd12.pcd", *clusters_pc);

  std::vector<Cluster> filtered_clusters;

  for (auto cluster : clusters) {
    if (std::all_of(_cone_validators_.begin(), _cone_validators_.end(), [&](const auto& validator) {
          return validator->coneValidator(&cluster, _ground_plane_);
        })) {
      filtered_clusters.push_back(cluster);
    }
  }

  for (auto cluster : clusters){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_after_icp(new pcl::PointCloud<pcl::PointXYZI>);
    auto fitness_score = _icp_->executeICP(cluster.get_point_cloud(), cluster_after_icp);
    if (cluster_after_icp->points.size() > 0)
        if (fitness_score >= 0){
          RCLCPP_DEBUG(this->get_logger(), "Fitness Score: %d", fitness_score);
          pcl::io::savePCDFileASCII("tentative.pcd", *cluster_after_icp);
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
  std::vector<custom_interfaces::msg::Cone> temp = {};
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
    temp.push_back(cone_message);
  }

  this->_cones_publisher->publish(message);
  this->cone_marker_array->publish(common_lib::communication::marker_array_from_structure_array(
      temp, "perception", "fsds/Lidar2"));
}
