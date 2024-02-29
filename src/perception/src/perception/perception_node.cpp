#include "perception/perception_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "adapter/fsds.hpp"
#include "adapter/testlidar.hpp"
#include "adapter/map.hpp"

#include <cstdio>
#include <string>
Perception::Perception(GroundRemoval* groundRemoval, Clustering* clustering,
            ConeDifferentiation* coneDifferentiator) :
            Node("perception"), groundRemoval(groundRemoval),
            clustering(clustering), coneDifferentiator(coneDifferentiator) {
  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>("cones", 10);

  this->adapter = adapter_map[mode](this);
}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    groundRemoval->groundRemoval(pcl_cloud, ground_removed_cloud);

    std::vector<Cluster> clusters;

    clustering->clustering(ground_removed_cloud, &clusters);

    RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
    RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
        pcl_cloud->points.size());
    RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
        ground_removed_cloud->points.size());
    RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters",
        clusters.size());

    for (int i = 0; i < clusters.size(); i++) {
        coneDifferentiator->coneDifferentiation(&clusters[i]);
        std::string color = clusters[i].getColor();
        RCLCPP_DEBUG(this->get_logger(), "Cone %d: %s", i, color.c_str());
    }

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
