#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cone_validator/displacement_validator.hpp>
#include <cone_validator/npoints_validator.hpp>

#include <cstdio>
#include <string>
#include <vector>

#include "common_lib/communication/marker.hpp"
#include "common_lib/config_load/config_load.hpp"
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

  params.adapter_ = global_config["global"]["adapter"].as<std::string>();
  params.vehicle_frame_id_ = global_config["global"]["vehicle_frame_id"].as<std::string>();

  const std::string perception_path =
      common_lib::config_load::get_config_yaml_path("perception", "perception", params.adapter_);
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Loading perception config from: %s",
               perception_path.c_str());
  const YAML::Node perception = YAML::LoadFile(perception_path);

  const auto perception_config = perception["perception"];
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Perception config contents: %s",
               YAML::Dump(perception_config).c_str());

  double fov_trim_angle = perception_config["fov_trim_angle"].as<double>();
  double pc_max_range = perception_config["pc_max_range"].as<double>();
  double pc_min_range = perception_config["pc_min_range"].as<double>();
  double pc_rlidar_max_height = perception_config["pc_rlidar_max_height"].as<double>();
  params.fov_trimming_ = std::make_shared<CutTrimming>(pc_max_range, pc_min_range, pc_rlidar_max_height, fov_trim_angle);

  std::string ground_removal_algorithm = perception_config["ground_removal"].as<std::string>();
  double ransac_epsilon = perception_config["ransac_epsilon"].as<double>();
  int ransac_iterations = perception_config["ransac_iterations"].as<int>();
  if (ground_removal_algorithm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_iterations);
  } else if (ground_removal_algorithm == "grid_ransac") {
    int n_angular_grids = perception_config["n_angular_grids"].as<int>();
    double radius_resolution = perception_config["radius_resolution"].as<double>();
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_iterations, n_angular_grids, radius_resolution);
  }

  int clustering_n_neighbours = perception_config["clustering_n_neighbours"].as<int>();
  double clustering_epsilon = perception_config["clustering_epsilon"].as<double>();
  params.clustering_ = std::make_shared<DBSCAN>(clustering_n_neighbours, clustering_epsilon);

  params.cone_differentiator_ = std::make_shared<LeastSquaresDifferentiation>();

  long unsigned int min_n_points = perception_config["min_n_points"].as<long unsigned int>();
  double min_height = perception_config["min_height"].as<double>();
  double large_max_height = perception_config["large_max_height"].as<double>();
  double small_max_height = perception_config["small_max_height"].as<double>();
  double height_cap = perception_config["height_cap"].as<double>();

  double min_xoy = perception_config["min_xoy"].as<double>();
  double max_xoy = perception_config["max_xoy"].as<double>();
  double min_z = perception_config["min_z"].as<double>();
  double max_z = perception_config["max_z"].as<double>();

  double min_distance_x = perception_config["min_distance_x"].as<double>();
  double min_distance_y = perception_config["min_distance_y"].as<double>();
  double min_distance_z = perception_config["min_distance_z"].as<double>();

  double min_z_score_x = perception_config["min_z_score_x"].as<double>();
  double max_z_score_x = perception_config["max_z_score_x"].as<double>();
  double min_z_score_y = perception_config["min_z_score_y"].as<double>();
  double max_z_score_y = perception_config["max_z_score_y"].as<double>();

  double out_distance_cap = perception_config["out_distance_cap"].as<double>();

  auto cone_validators = std::make_shared<std::unordered_map<std::string, std::shared_ptr<ConeValidator>>>(
    std::unordered_map<std::string, std::shared_ptr<ConeValidator>>{
      {"npoints", std::make_shared<NPointsValidator>(min_n_points)},
      {"height", std::make_shared<HeightValidator>(min_height, large_max_height, small_max_height, height_cap)},
      {"cylinder", std::make_shared<CylinderValidator>(0.228, 0.325, 0.285, 0.505, out_distance_cap)},
      {"deviation", std::make_shared<DeviationValidator>(min_xoy, max_xoy, min_z, max_z)},
      {"displacement", std::make_shared<DisplacementValidator>(min_distance_x, min_distance_y, min_distance_z)},
      {"zscore", std::make_shared<ZScoreValidator>(min_z_score_x, max_z_score_x, min_z_score_y, max_z_score_y)}
    });

  double height_out_weight = perception_config["height_out_weight"].as<double>();
  double height_in_weight = perception_config["height_in_weight"].as<double>();
  double cylinder_radius_weight = perception_config["cylinder_radius_weight"].as<double>();
  double cylinder_height_weight = perception_config["cylinder_height_weight"].as<double>();
  double cylinder_npoints_weight = perception_config["cylinder_npoints_weight"].as<double>();
  double npoints_weight = perception_config["npoints_weight"].as<double>();
  double displacement_x_weight = perception_config["displacement_x_weight"].as<double>();
  double displacement_y_weight = perception_config["displacement_y_weight"].as<double>();
  double displacement_z_weight = perception_config["displacement_z_weight"].as<double>();
  double deviation_xoy_weight = perception_config["deviation_xoy_weight"].as<double>();
  double deviation_z_weight = perception_config["deviation_z_weight"].as<double>();

  auto weight_values = std::make_shared<std::unordered_map<std::string, double>>(
    std::unordered_map<std::string, double>{
      {"height_out_weight", height_out_weight},
      {"height_in_weight", height_in_weight},
      {"cylinder_radius_weight", cylinder_radius_weight},
      {"cylinder_height_weight", cylinder_height_weight},
      {"cylinder_npoints_weight", cylinder_npoints_weight},
      {"npoints_weight", npoints_weight},
      {"displacement_x_weight", displacement_x_weight},
      {"displacement_y_weight", displacement_y_weight},
      {"displacement_z_weight", displacement_z_weight},
      {"deviation_xoy_weight", deviation_xoy_weight},
      {"deviation_z_weight", deviation_z_weight}
    });

  double weight_sum = 0.0;
  for (const auto &pair : *weight_values) {
    weight_sum += pair.second;
  }
  for (auto &pair : *weight_values) {
    pair.second /= weight_sum;
  }

  double min_confidence = perception_config["min_confidence"].as<double>();

  params.cone_evaluator_ = std::make_shared<ConeEvaluator>(cone_validators, weight_values, min_confidence);

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
