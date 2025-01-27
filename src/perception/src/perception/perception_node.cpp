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
#include "yaml-cpp/yaml.h"

std_msgs::msg::Header header;

const std::unordered_map<std::string, std::string> adapter_frame_map = {
    {"vehicle", "lidar"},
    {"eufs", "velodyne"},
    {"fsds", "lidar"},
    {"vehicle_preprocessed", "lidar"},
    {"fst", "lidar"}};

PerceptionParameters Perception::load_config() {
  PerceptionParameters params;

  const std::string global_config_path =
      common_lib::config_load::get_config_yaml_path("perception", "global", "global_config");
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Loading global config from: %s",
               global_config_path.c_str());
  const YAML::Node global_config = YAML::LoadFile(global_config_path);

  const std::string adapter = global_config["global"]["adapter"].as<std::string>();
  params.adapter_ = adapter;
  params.vehicle_frame_id_ = (adapter == "eufs") ? "velodyne" : "hesai_lidar";

  const std::string perception_path =
      common_lib::config_load::get_config_yaml_path("perception", "perception", adapter);
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Loading perception config from: %s",
               perception_path.c_str());
  const YAML::Node perception = YAML::LoadFile(perception_path);

  const auto perception_config = perception["perception"];
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Perception config contents: %s",
               YAML::Dump(perception_config).c_str());

  params.pc_max_range_ = perception_config["pc_max_range"].as<double>();

  const std::string ground_removal_algorithm =
      perception_config["ground_removal"].as<std::string>();
  if (ground_removal_algorithm == "ransac") {
    params.ground_removal_ =
        std::make_shared<RANSAC>(perception_config["ransac_epsilon"].as<double>(),
                                 perception_config["ransac_n_neighbours"].as<int>());
  } else if (ground_removal_algorithm == "grid_ransac") {
    params.ground_removal_ =
        std::make_shared<GridRANSAC>(perception_config["ransac_epsilon"].as<double>(),
                                     perception_config["ransac_n_neighbours"].as<int>(),
                                     perception_config["n_angular_grids"].as<int>(),
                                     perception_config["radius_resolution"].as<double>());
  }

  params.clustering_ =
      std::make_shared<DBSCAN>(perception_config["clustering_n_neighbours"].as<int>(),
                               perception_config["clustering_epsilon"].as<double>());

  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();

  if (params.adapter_ != "eufs") {
    params.cone_validators_ = {
        std::make_shared<CylinderValidator>(0.200, 0.325),
        std::make_shared<HeightValidator>(perception_config["min_height"].as<double>(),
                                          perception_config["max_height"].as<double>()),
        std::make_shared<DeviationValidator>(
            perception_config["min_xoy"].as<double>(), perception_config["max_xoy"].as<double>(),
            perception_config["min_z"].as<double>(), perception_config["max_z"].as<double>()),
        std::make_shared<ZScoreValidator>(perception_config["min_z_score_x"].as<double>(),
                                          perception_config["max_z_score_x"].as<double>(),
                                          perception_config["min_z_score_y"].as<double>(),
                                          perception_config["max_z_score_y"].as<double>())};
  }

  params.distance_predict_ =
      std::make_shared<DistancePredict>(perception_config["vertical_resolution"].as<double>(),
                                        perception_config["horizontal_resolution"].as<double>());

  params.icp_ = std::make_shared<ICP>(perception_config["target_file"].as<std::string>(),
                                      perception_config["max_correspondence_distance"].as<double>(),
                                      perception_config["max_iteration"].as<int>(),
                                      perception_config["transformation_epsilon"].as<double>(),
                                      perception_config["euclidean_fitness_epsilon"].as<double>());

  params.fov_trim_ = perception_config["fov_trim"].as<double>();

  return params;
}

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _vehicle_frame_id_(params.vehicle_frame_id_),
      _fov_trimming_(params.fov_trimming_),
      _ground_removal_(params.ground_removal_),
      _clustering_(params.clustering_),
      _cone_differentiator_(params.cone_differentiator_),
      _cone_evaluator_(params.cone_evaluator_),
      _icp_(params.icp_) {
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/perception/cones", 10);

  this->_ground_removed_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_removed_cloud", 10);

  this->_perception_execution_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float64>("/perception/execution_time", 10);

  // Determine which adapter is being used
  std::unordered_map<std::string, std::tuple<std::string, rclcpp::QoS>> adapter_topic_map = {
      {"vehicle", {"/lidar_points", rclcpp::QoS(10)}},
      {"eufs",
       {"/velodyne_points", rclcpp::QoS(1).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)}},
      {"fsds", {"/lidar/Lidar1", rclcpp::QoS(10)}},
      {"vehicle_preprocessed", {"/rslidar_points/pre_processed", rclcpp::QoS(10)}},
      {"fst", {"/hesai/pandar", rclcpp::QoS(10)}}};

  try {
    this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        std::get<0>(adapter_topic_map.at(params.adapter_)),
        std::get<1>(adapter_topic_map.at(params.adapter_)),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->point_cloud_callback(msg);
        });
  } catch (const std::out_of_range& e) {
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

  // Pass-trough Filter (trim Pcl)
  _fov_trimming_->fov_trimming(pcl_cloud);

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

  for (auto& cluster : clusters) {
    if (_cone_evaluator_->evaluateCluster(cluster, _ground_plane_)) {
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
  RCLCPP_DEBUG(this->get_logger(), "Point Cloud after Clustering: %ld clusters", clusters.size());
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
    cone_message.is_large = cones->at(i).get_is_large();
    cone_message.confidence = cones->at(i).get_confidence();
    message.cone_array.push_back(cone_message);
    message_array.push_back(cone_message);
  }

  this->_cones_publisher->publish(message);
  // TODO: correct frame id to LiDAR instead of vehicle
  this->_cone_marker_array_->publish(common_lib::communication::marker_array_from_structure_array(
      message_array, "perception", this->_vehicle_frame_id_, "green"));
}