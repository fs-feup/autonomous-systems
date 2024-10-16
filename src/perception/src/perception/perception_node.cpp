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

const std::unordered_map<std::string, std::string> adapter_frame_map = {
    {"vehicle", "lidar"},
    {"eufs", "velodyne"},
    {"fsds", "lidar"},
    {"vehicle_preprocessed", "lidar"},
    {"fst", "lidar"}};

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _vehicle_frame_id_(params.vehicle_frame_id_),
      _ground_removal_(params.ground_removal_),
      _clustering_(params.clustering_),
      _cone_differentiator_(params.cone_differentiator_),
      _cone_validators_(params.cone_validators_),
      _cone_evaluator_(
          params.distance_predict_), /* This is probably wrong and will give an eror when running
                                        but I dunno the right types to use, davide fix pls */
      _icp_(params.icp_) {
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

  this->_ground_removed_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_removed_cloud", 10);

  this->_perception_execution_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/perception/execution_time", 10);

  this->_fov_trim_ = params.fov_trim_;

  this->_pc_max_range_ = params.pc_max_range_;

  // std::unordered_map<std::string, std::tuple<std::string, rclcpp::QoS>> adapter_topic_map = {
  //     {"vehicle", {"/rslidar_points", rclcpp::QoS(10)}},
  //     {"eufs",
  //      {"/velodyne_points", rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)}},
  //     {"fsds", {"/lidar/Lidar1", rclcpp::QoS(10)}},
  //     {"vehicle_preprocessed", {"/rslidar_points/pre_processed", rclcpp::QoS(10)}},
  //     {"fst", {"/hesai/pandar", rclcpp::QoS(10)}}};

  // this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     std::get<0>(adapter_topic_map[params.adapter_]),
  //     std::get<1>(adapter_topic_map[params.adapter_]),
  //     [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  //       this->point_cloud_callback(msg);
  //     });

  // TODO: fix this shit
  this->_adapter_ = params.adapter_;
  if (params.adapter_ == "vehicle") {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } else if (params.adapter_ == "eufs") {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } else if (params.adapter_ == "fsds") {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/Lidar1", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } else if (params.adapter_ == "vehicle_preprocessed") {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points/pre_processed", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } else if (params.adapter_ == "fst") {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } else {
    RCLCPP_ERROR(this->get_logger(), "Adapter not recognized: %s", params.adapter_.c_str());
  }

  this->_cone_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/perception/visualization/cones", 10);

  RCLCPP_INFO(this->get_logger(), "Perception Node created with adapter: %s",
              params.adapter_.c_str());
}

void Perception::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  rclcpp::Time time = this->now();

  // Message Read
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  header = (*msg).header;
  pcl::fromROSMsg(*msg, *pcl_cloud);

  // Pass-trough Filter
  fov_trimming(pcl_cloud, this->_pc_max_range_, -_fov_trim_, _fov_trim_, 0);

  // Ground Removal
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  _ground_removal_->ground_removal(pcl_cloud, ground_removed_cloud, _ground_plane_);

  // Debugging utils -> Useful to check the ground removed point cloud
  sensor_msgs::msg::PointCloud2 ground_remved_msg;
  pcl::toROSMsg(*ground_removed_cloud, ground_remved_msg);
  ground_remved_msg.header.frame_id = _vehicle_frame_id_;
  this->_ground_removed_publisher_->publish(ground_remved_msg);

  // Clustering
  std::vector<Cluster> clusters;
  _clustering_->clustering(ground_removed_cloud, &clusters);


  // Z-scores calculation for future validations
  Cluster::set_z_scores(clusters);

  // Filtering
  std::vector<Cluster> filtered_clusters;

  for (auto cluster : clusters) {
    bool is_valid = true;
    for (auto validator : _cone_validators_){
      is_valid = is_valid && validator->coneValidator(&cluster, _ground_plane_);

      // Break the cycle to avoid wasting time on invalid clusters
      if (!is_valid) break;
    }
    if (is_valid) {
      filtered_clusters.push_back(cluster);
    }
  }

  // Execution Time calculation
  rclcpp::Time end_time = this->now();
  std_msgs::msg::Float64 perception_execution_time;
  perception_execution_time.data = (end_time - time).seconds() * 1000;
  this->_perception_execution_time_publisher_->publish(perception_execution_time);

  // Logging
  RCLCPP_DEBUG(this->get_logger(), "---------- Point Cloud Received ----------");
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud Before Ground Removal: %ld points",
               pcl_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud After Ground Removal: %ld points",
               ground_removed_cloud->points.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters",
               clusters.size());
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Validations: %ld clusters",
               filtered_clusters.size());
  

  publish_cones(&filtered_clusters);
}

void Perception::publish_cones(std::vector<Cluster>* cones) {
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
  // TODO: correct frame id to LiDAR instead of vehicle
  this->_cone_marker_array_->publish(common_lib::communication::marker_array_from_structure_array(
      message_array, "perception", this->_vehicle_frame_id_, "green"));
}

void Perception::fov_trimming(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double max_distance,
                              double min_angle, double max_angle, double x_discount) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  double center_x = -x_discount; // assuming (0, 0) as the center

  for (const auto& point : cloud->points) {
    // Calculate distance from the origin (assuming the sensor is at the origin)
    double distance = std::sqrt((point.x - center_x) * (point.x - center_x) + point.y * point.y);

    // Calculate the angle in the XY plane
    double angle =
        std::atan2(point.y, point.x - center_x) * 180 / M_PI;  // get angle and convert in to degrees

    if (distance <= 0.5) {  // Ignore points from the vehicle
      continue;
    }

    if (point.z >= -0.2){
      continue;
    }

    // Check if the point is within the specified distance and angle range
    if (distance <= max_distance && angle >= min_angle && angle <= max_angle) {
      trimmed_cloud->points.push_back(point);
    }
  }

  // Replace the input cloud with the trimmed cloud
  *cloud = *trimmed_cloud;
}