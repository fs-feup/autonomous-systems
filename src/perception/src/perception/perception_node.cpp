#include "perception/perception_node.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cone_validator/displacement_validator.hpp>
#include <cone_validator/npoints_validator.hpp>
#include <cstdio>
#include <string>
#include <utils/trimming_parameters.hpp>
#include <vector>

#include "common_lib/communication/marker.hpp"
#include "common_lib/competition_logic/mission_logic.hpp"
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

  if (params.adapter_ == "pacsim") {
    params.adapter_ = "vehicle";
  }

  const std::string perception_path =
      common_lib::config_load::get_config_yaml_path("perception", "perception", params.adapter_);
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Loading perception config from: %s",
               perception_path.c_str());
  const YAML::Node perception = YAML::LoadFile(perception_path);

  const auto perception_config = perception["perception"];
  RCLCPP_DEBUG(rclcpp::get_logger("perception"), "Perception config contents: %s",
               YAML::Dump(perception_config).c_str());

  const auto default_mission_str = perception_config["default_mission"].as<std::string>();
  params.default_mission_ =
      static_cast<uint8_t>(common_lib::competition_logic::fsds_to_system.at(default_mission_str));

  TrimmingParameters trim_params;
  trim_params.lidar_rotation = perception_config["lidar_rotation"].as<double>();
  trim_params.lidar_pitch = perception_config["lidar_pitch"].as<double>();
  trim_params.min_range = perception_config["min_range"].as<double>();
  trim_params.max_height = perception_config["max_height"].as<double>();
  trim_params.lidar_height = perception_config["lidar_height"].as<double>();

  trim_params.max_range = perception_config["max_range"].as<double>();
  trim_params.fov_trim_angle = perception_config["fov_trim_angle"].as<double>();
  trim_params.split_params.n_angular_grids = perception_config["n_angular_grids"].as<int>();
  trim_params.split_params.radius_resolution = perception_config["radius_resolution"].as<double>();
  trim_params.split_params.fov_angle = 2 * trim_params.fov_trim_angle;

  trim_params.acc_max_range = perception_config["acc_max_range"].as<double>();
  trim_params.acc_fov_trim_angle = perception_config["acc_fov_trim_angle"].as<double>();
  trim_params.acc_max_y = perception_config["acc_max_y"].as<double>();
  trim_params.acc_split_params.n_angular_grids = perception_config["acc_n_angular_grids"].as<int>();
  trim_params.acc_split_params.radius_resolution =
      perception_config["acc_radius_resolution"].as<double>();
  trim_params.acc_split_params.fov_angle = 2 * trim_params.acc_fov_trim_angle;

  trim_params.skid_max_range = perception_config["skid_max_range"].as<double>();
  const double min_distance_to_cone = perception_config["skid_min_distance_to_cone"].as<double>();
  trim_params.skid_fov_trim_angle =
      90 - std::acos(1.5 / std::max(min_distance_to_cone, 1.5)) * 180 / M_PI;
  trim_params.skid_split_params.n_angular_grids =
      perception_config["skid_n_angular_grids"].as<int>();
  trim_params.skid_split_params.radius_resolution =
      perception_config["skid_radius_resolution"].as<double>();
  trim_params.skid_split_params.fov_angle = 2 * trim_params.skid_fov_trim_angle;

  auto acceleration_trimming = std::make_shared<AccelerationTrimming>(trim_params);
  auto skidpad_trimming = std::make_shared<SkidpadTrimming>(trim_params);
  auto cut_trimming = std::make_shared<CutTrimming>(trim_params);

  auto temp_fov_trim_map = std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>{
      {static_cast<int16_t>(Mission::ACCELERATION), acceleration_trimming},
      {static_cast<int16_t>(Mission::SKIDPAD), skidpad_trimming},
      {static_cast<int16_t>(Mission::TRACKDRIVE), cut_trimming},
      {static_cast<int16_t>(Mission::AUTOCROSS), acceleration_trimming},
      {static_cast<int16_t>(Mission::EBS_TEST), cut_trimming},
      {static_cast<int16_t>(Mission::INSPECTION), cut_trimming},
      {static_cast<int16_t>(Mission::EBS_TEST), acceleration_trimming}};

  params.fov_trim_map_ =
      std::make_shared<std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>>(
          temp_fov_trim_map);

  std::string ground_removal_algorithm = perception_config["ground_removal"].as<std::string>();
  double ransac_epsilon = perception_config["ransac_epsilon"].as<double>();
  int ransac_iterations = perception_config["ransac_iterations"].as<int>();
  if (ground_removal_algorithm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_iterations);
  } else if (ground_removal_algorithm == "grid_ransac") {
    params.ground_removal_ = std::make_shared<GridRANSAC>(ransac_epsilon, ransac_iterations);
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

  // Evaluator Parameters (ConeValidators + weights + minimum confidence)
  auto eval_params = std::make_shared<EvaluatorParameters>();

  // ConeValidators for cone evaluator
  eval_params->npoints_validator = std::make_shared<NPointsValidator>(min_n_points);
  eval_params->height_validator =
      std::make_shared<HeightValidator>(min_height, large_max_height, small_max_height, height_cap);
  eval_params->cylinder_validator =
      std::make_shared<CylinderValidator>(0.23, 0.33, 0.228, 0.325, out_distance_cap);
  eval_params->deviation_validator =
      std::make_shared<DeviationValidator>(min_xoy, max_xoy, min_z, max_z);
  eval_params->displacement_validator =
      std::make_shared<DisplacementValidator>(min_distance_x, min_distance_y, min_distance_z);
  eval_params->zscore_validator =
      std::make_shared<ZScoreValidator>(min_z_score_x, max_z_score_x, min_z_score_y, max_z_score_y);

  // Weight values for cone evaluator
  eval_params->height_out_weight = perception_config["height_out_weight"].as<double>();
  eval_params->height_in_weight = perception_config["height_in_weight"].as<double>();
  eval_params->cylinder_radius_weight = perception_config["cylinder_radius_weight"].as<double>();
  eval_params->cylinder_height_weight = perception_config["cylinder_height_weight"].as<double>();
  eval_params->cylinder_npoints_weight = perception_config["cylinder_npoints_weight"].as<double>();
  eval_params->npoints_weight = perception_config["npoints_weight"].as<double>();
  eval_params->displacement_x_weight = perception_config["displacement_x_weight"].as<double>();
  eval_params->displacement_y_weight = perception_config["displacement_y_weight"].as<double>();
  eval_params->displacement_z_weight = perception_config["displacement_z_weight"].as<double>();
  eval_params->deviation_xoy_weight = perception_config["deviation_xoy_weight"].as<double>();
  eval_params->deviation_z_weight = perception_config["deviation_z_weight"].as<double>();

  eval_params->normalize_weights();

  // Minimum confidence needed for a cluster to be considered a cone.
  eval_params->min_confidence = perception_config["min_confidence"].as<double>();

  params.cone_evaluator_ = params.cone_evaluator_ = std::make_shared<ConeEvaluator>(eval_params);

  return params;
}

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _vehicle_frame_id_(params.vehicle_frame_id_),
      _mission_type_(params.default_mission_),
      _fov_trim_map_(params.fov_trim_map_),
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

  this->_operational_status_subscription =
      this->create_subscription<custom_interfaces::msg::OperationalStatus>(
          "/vehicle/operational_status", rclcpp::QoS(10),
          [this](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
            _mission_type_ = msg->as_mission;
          });

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
  const SplitParameters split_params = _fov_trim_map_->at(_mission_type_)->fov_trimming(pcl_cloud);

  // Ground Removal
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_removed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  _ground_removal_->ground_removal(pcl_cloud, ground_removed_cloud, _ground_plane_, split_params);

  // Debugging utils -> Useful to check the ground removed point cloud
  sensor_msgs::msg::PointCloud2 ground_removed_msg;
  pcl::toROSMsg(*ground_removed_cloud, ground_removed_msg);
  ground_removed_msg.header.frame_id = _vehicle_frame_id_;
  this->_ground_removed_publisher_->publish(ground_removed_msg);

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
