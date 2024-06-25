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
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

  this->_ground_removed_publisher_ = 
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_removed_cloud", 10);

  std::unordered_map<std::string, std::string> adapter_topic_map = {
      {"vehicle", "/hesai/pandar"}, {"eufs", "/velodyne_points"}, {"fsds", "/lidar/Lidar1"}, 
          {"robosense", "/rslidar_points/pre_processed"}};

  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      adapter_topic_map[params.adapter_], 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointCloudCallback(msg);
      });
  
  this->_cone_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>
          ("/perception/visualization/cones", 10);

  RCLCPP_INFO(this->get_logger(), "Perception Node created with adapter: %s",
              params.adapter_.c_str());
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  // Message Read
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  header = (*msg).header;
  pcl::fromROSMsg(*msg, *pcl_cloud);

  // Ground Removal
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  _ground_removal_->ground_removal(pcl_cloud, ground_removed_cloud, _ground_plane_);

  // Debugging utils -> Useful to check the ground removed point cloud
  sensor_msgs::msg::PointCloud2 ground_remved_msg;
  pcl::toROSMsg(*ground_removed_cloud, ground_remved_msg);
  this->_ground_removed_publisher_->publish(ground_remved_msg);

  // Clustering
  std::vector<Cluster> clusters;
  _clustering_->clustering(ground_removed_cloud, &clusters);

  // Filtering
  std::vector<Cluster> filtered_clusters;
  for (auto cluster : clusters) {
    // Temporary: Just the first validation
    if (_cone_validators_[0]->coneValidator(&cluster, _ground_plane_)){
      filtered_clusters.push_back(cluster);
    }
  }

  // Logging
  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", 
               filtered_clusters.size());

  publishCones(&filtered_clusters);
}

void Perception::publishCones(std::vector<Cluster>* cones) {
  auto message = custom_interfaces::msg::ConeArray();
  std::vector<custom_interfaces::msg::Cone> message_array = {};
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
    message_array.push_back(cone_message);
  }

  this->_cones_publisher->publish(message);
  this->_cone_marker_array_->publish(common_lib::communication::marker_array_from_structure_array(
      message_array, "perception", "rslidar", "yellow"));
}
