#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdio>
#include <string>
#include <vector>
#include "common_lib/communication/marker.hpp"
#include "std_msgs/msg/header.hpp"

std_msgs::msg::Header header;

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _ground_removal_(params.ground_removal_),
      _clustering_(params.clustering_),
      _cone_differentiator_(params.cone_differentiator_),
      _cone_validators_(params.cone_validators_),
      _cone_evaluator_(
          params.distance_predict_),  /* This is probably wrong and will give an eror when running
                                         but I dunno the right types to use, davide fix pls */
      _icp_(params.icp_){
        
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/ground_truth", 10);

  std::unordered_map<std::string, std::string> adapter_topic_map = {
      {"vehicle", "rslidar_points"}, {"eufs", "/velodyne_points"}, {"fsds", "/lidar/Lidar1"}};

  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      adapter_topic_map[params.adapter_], 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointCloudCallback(msg);
      });
  
  this->cone_marker_array = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception/visualization/ground_truth/cones", 10);

  this->ground_removed_pc = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points/pre_processed", 10);

  RCLCPP_INFO(this->get_logger(), "Perception Node created with adapter: %s",
              params.adapter_.c_str());
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  header = (*msg).header;

  // TODO: vscode is complaining here for some reason about template not matching argument list
  pcl::fromROSMsg(*msg, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for (const auto& point : pcl_cloud->points) {
    if (point.x >= 1 && point.y >= -1.8 && point.x < 14 && point.y <= 9) {
      processed_cloud->points.push_back(point);
    }
  }

  // Ground Removal
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  _ground_removal_->ground_removal(processed_cloud, ground_removed_cloud, _ground_plane_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr finite_ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);


  // Processed Point Cloud
  sensor_msgs::msg::PointCloud2 finite_ground_removed_msg;
  pcl::toROSMsg(*processed_cloud, finite_ground_removed_msg);
  finite_ground_removed_msg.header = header;
  this->ground_removed_pc->publish(finite_ground_removed_msg);



  for (const auto& point : ground_removed_cloud->points) {
    if (pcl::isFinite(point)) {
      finite_ground_removed_cloud->points.push_back(point);
    }
  }
  finite_ground_removed_cloud->width = finite_ground_removed_cloud->points.size();
  finite_ground_removed_cloud->height = 1;
  finite_ground_removed_cloud->is_dense = true;

  // Clustering
  std::vector<Cluster> clusters;
  _clustering_->clustering(finite_ground_removed_cloud, &clusters);

  // Filtering
  std::vector<Cluster> filtered_clusters;
  for (auto cluster : clusters) {
    if (std::all_of(_cone_validators_.begin(), _cone_validators_.end(), [&](const auto& validator) {
          return validator->coneValidator(&cluster, _ground_plane_);
        })) {
      filtered_clusters.push_back(cluster);
    }
  }

  // Logging
  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", clusters.size());

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
  }
  message.cone_array = {};

  auto cone_message = custom_interfaces::msg::Cone();
  cone_message.position.x = 2.174;
  cone_message.position.y = -1.614;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 5.302;
  cone_message.position.y = -1.614;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 8.43;
  cone_message.position.y = -1.614;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 11.008;
  cone_message.position.y = -0.286;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 11.946;
  cone_message.position.y = 2.842;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 11.946;
  cone_message.position.y = 5.97;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 11.946;
  cone_message.position.y = 8.798;
  cone_message.confidence = 1;
  cone_message.color = "blue";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 2.174;
  cone_message.position.y = 1.614;
  cone_message.confidence = 1;
  cone_message.color = "yellow";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 5.302;
  cone_message.position.y = 1.614;
  cone_message.confidence = 1;
  cone_message.color = "yellow";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 8.67;
  cone_message.position.y = 2.592;
  cone_message.confidence = 1;
  cone_message.color = "yellow";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 8.67;
  cone_message.position.y = 5.72;
  cone_message.confidence = 1;
  cone_message.color = "yellow";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  cone_message.position.x = 8.67;
  cone_message.position.y = 8.548;
  cone_message.confidence = 1;
  cone_message.color = "yellow";
  temp.push_back(cone_message);
  message.cone_array.push_back(cone_message);

  this->_cones_publisher->publish(message);
  this->cone_marker_array->publish(common_lib::communication::marker_array_from_structure_array(
      temp, "perception", "rslidar", "red", "cylinder", 0.5, visualization_msgs::msg::Marker::MODIFY));
}
