#include "perception/perception_node.hpp"

static const std_msgs::msg::Header header;

static const inline std::unordered_map<std::string, std::string> adapter_frame_map = {
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

  // FOV Trimming Parameters
  TrimmingParameters trim_params;
  trim_params.min_range = perception_config["min_range"].as<double>();
  trim_params.max_height = perception_config["max_height"].as<double>();
  trim_params.max_range = perception_config["max_range"].as<double>();
  trim_params.acc_max_range = perception_config["acc_max_range"].as<double>();
  trim_params.acc_max_y = perception_config["acc_max_y"].as<double>();
  trim_params.skid_max_range = perception_config["skid_max_range"].as<double>();
  trim_params.lidar_height = perception_config["lidar_height"].as<double>();
  trim_params.lidar_horizontal_resolution =
      perception_config["lidar_horizontal_resolution"].as<double>();
  trim_params.lidar_vertical_resolution =
      perception_config["lidar_vertical_resolution"].as<double>();
  trim_params.apply_rotation = perception_config["apply_rotation"].as<bool>();
  trim_params.rotation = perception_config["rotation"].as<double>();
  trim_params.apply_fov_trimming = perception_config["apply_fov_trimming"].as<bool>();
  trim_params.fov = perception_config["fov"].as<double>();

  auto acceleration_trimming = std::make_shared<AccelerationTrimming>(trim_params);
  auto skidpad_trimming = std::make_shared<SkidpadTrimming>(trim_params);
  auto cut_trimming = std::make_shared<CutTrimming>(trim_params);

  auto temp_fov_trim_map = std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>{
      {static_cast<int16_t>(Mission::MANUAL), cut_trimming},
      {static_cast<int16_t>(Mission::ACCELERATION), acceleration_trimming},
      {static_cast<int16_t>(Mission::SKIDPAD), skidpad_trimming},
      {static_cast<int16_t>(Mission::TRACKDRIVE), cut_trimming},
      {static_cast<int16_t>(Mission::AUTOCROSS), cut_trimming},
      {static_cast<int16_t>(Mission::INSPECTION), cut_trimming},
      {static_cast<int16_t>(Mission::EBS_TEST), acceleration_trimming},
      {static_cast<int16_t>(Mission::MANUAL), cut_trimming},
      {static_cast<int16_t>(Mission::NONE), cut_trimming}};

  params.fov_trim_map_ =
      std::make_shared<std::unordered_map<int16_t, std::shared_ptr<FovTrimming>>>(
          temp_fov_trim_map);

  // Ground Grid Parameters
  double ground_grid_range = perception_config["ground_grid_range"].as<double>();
  double ground_grid_angle = perception_config["ground_grid_angle"].as<double>();
  double ground_grid_radius = perception_config["ground_grid_radius"].as<double>();
  double ground_grid_start_augmentation =
      perception_config["ground_grid_start_augmentation"].as<double>();
  double ground_grid_radius_augmentation =
      perception_config["ground_grid_radius_augmentation"].as<double>();
  params.ground_grid_ = std::make_shared<GroundGrid>(
      ground_grid_range, ground_grid_angle, ground_grid_radius, ground_grid_start_augmentation,
      ground_grid_radius_augmentation, trim_params.fov);

  // Ground Removal Parameters
  std::string ground_removal_algorithm = perception_config["ground_removal"].as<std::string>();
  double ransac_epsilon = perception_config["ransac_epsilon"].as<double>();
  int ransac_iterations = perception_config["ransac_iterations"].as<int>();
  double himmelsbach_grid_angle = perception_config["himmelsbach_grid_angle"].as<double>();
  double himmelsbach_max_slope = perception_config["himmelsbach_max_slope"].as<double>();
  double himmelsbach_alpha = perception_config["himmelsbach_initial_alpha"].as<double>();
  double himmelsbach_alpha_augmentation_m =
      perception_config["himmelsbach_alpha_augmentation_m"].as<double>();
  double himmelsbach_start_augmentation =
      perception_config["himmelsbach_start_augmentation"].as<double>();

  if (ground_removal_algorithm == "ransac") {
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_iterations);
  } else if (ground_removal_algorithm == "himmelsbach") {
    params.ground_removal_ = std::make_shared<Himmelsbach>(
        himmelsbach_grid_angle, himmelsbach_max_slope, himmelsbach_alpha,
        himmelsbach_alpha_augmentation_m, himmelsbach_start_augmentation, trim_params);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("perception"),
                 "Ground removal algorithm not recognized: %s, using RANSAC as default",
                 ground_removal_algorithm.c_str());
    params.ground_removal_ = std::make_shared<RANSAC>(ransac_epsilon, ransac_iterations);
  }

  // Wall Removal Parameters
  double wall_removal_grid_angle = perception_config["wall_removal_grid_angle"].as<double>();
  double wall_removal_grid_radius = perception_config["wall_removal_grid_radius"].as<double>();
  double wall_removal_start_augmentation =
      perception_config["wall_removal_start_augmentation"].as<double>();
  double wall_removal_radius_augmentation =
      perception_config["wall_removal_radius_augmentation"].as<double>();
  int wall_removal_max_points_per_cluster =
      perception_config["wall_removal_max_points_per_cluster"].as<int>();
  params.wall_removal_ = std::make_shared<GridWallRemoval>(
      wall_removal_grid_angle, wall_removal_grid_radius, wall_removal_start_augmentation,
      wall_removal_radius_augmentation, trim_params.fov, wall_removal_max_points_per_cluster);

  // Clustering Parameters
  std::string clustering_algorithm = perception_config["clustering"].as<std::string>();
  int DBSCAN_clustering_n_neighbours = perception_config["DBSCAN_n_neighbours"].as<int>();
  double DBSCAN_clustering_epsilon = perception_config["DBSCAN_clustering_epsilon"].as<double>();
  double grid_clustering_grid_angle = perception_config["grid_clustering_grid_angle"].as<double>();
  double grid_clustering_grid_radius =
      perception_config["grid_clustering_grid_radius"].as<double>();
  double grid_clustering_start_augmentation =
      perception_config["grid_clustering_start_augmentation"].as<double>();
  double grid_clustering_radius_augmentation =
      perception_config["grid_clustering_radius_augmentation"].as<double>();

  if (clustering_algorithm == "DBSCAN") {
    params.clustering_ =
        std::make_shared<DBSCAN>(DBSCAN_clustering_n_neighbours, DBSCAN_clustering_epsilon);
  } else if (clustering_algorithm == "GridClustering") {
    params.clustering_ = std::make_shared<GridClustering>(
        grid_clustering_grid_angle, grid_clustering_grid_radius, grid_clustering_start_augmentation,
        grid_clustering_radius_augmentation, trim_params.fov);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("perception"),
                 "Clustering algorithm not recognized: %s, using DBSCAN as default",
                 clustering_algorithm.c_str());
    params.clustering_ =
        std::make_shared<DBSCAN>(DBSCAN_clustering_n_neighbours, DBSCAN_clustering_epsilon);
  }

  // Cone Evaluator Parameters
  auto eval_params = std::make_shared<EvaluatorParameters>();
  eval_params->small_cone_width = perception_config["small_cone_width"].as<double>();
  eval_params->large_cone_width = perception_config["large_cone_width"].as<double>();
  eval_params->small_cone_height = perception_config["small_cone_height"].as<double>();
  eval_params->large_cone_height = perception_config["large_cone_height"].as<double>();
  eval_params->n_out_points_ratio = perception_config["n_out_points_ratio"].as<double>();
  eval_params->max_distance_from_ground_min =
      perception_config["max_distance_from_ground_min"].as<double>();
  eval_params->max_distance_from_ground_max =
      perception_config["max_distance_from_ground_max"].as<double>();
  eval_params->lidar_height = trim_params.lidar_height;
  eval_params->lidar_vertical_resolution = trim_params.lidar_vertical_resolution;
  eval_params->lidar_horizontal_resolution = trim_params.lidar_horizontal_resolution;
  eval_params->visibility_factor = perception_config["visibility_factor"].as<double>();
  eval_params->expected_points_threshold =
      perception_config["expected_points_threshold"].as<double>();

  params.cone_evaluator_ = std::make_shared<ConeEvaluator>(eval_params);
  return params;
}

Perception::Perception(const PerceptionParameters& params)
    : Node("perception"),
      _vehicle_frame_id_(params.vehicle_frame_id_),
      _mission_type_(params.default_mission_),
      _fov_trim_map_(params.fov_trim_map_),
      _ground_grid_(params.ground_grid_),
      _ground_removal_(params.ground_removal_),
      _wall_removal_(params.wall_removal_),
      _clustering_(params.clustering_),
      _cone_differentiator_(params.cone_differentiator_),
      _cone_evaluator_(params.cone_evaluator_),
      _icp_(params.icp_) {
  this->_cones_publisher =
      this->create_publisher<custom_interfaces::msg::PerceptionOutput>("/perception/cones", 10);

  this->_ground_removed_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground_removed_cloud", 10);

  this->_wall_removed_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/wall_removed_cloud", 10);

  this->_perception_execution_time_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/perception/execution_time", 10);
  this->_execution_times_ = std::make_shared<std::vector<double>>(7, 0.0);

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

  this->_velocities_subscription_ = this->create_subscription<custom_interfaces::msg::Velocities>(
      "/state_estimation/velocities", rclcpp::QoS(10),
      std::bind(&Perception::velocities_callback, this, std::placeholders::_1));
  this->_cone_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/perception/visualization/cones", 10);

  this->lidar_off_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&Perception::lidar_timer_callback, this));

  emergency_client_ = this->create_client<std_srvs::srv::Trigger>("/as_srv/emergency");

  RCLCPP_INFO(this->get_logger(), "Perception Node created with adapter: %s",
              params.adapter_.c_str());
}

void Perception::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Get start perception time
  // rclcpp::Time pcl_time = msg->header.stamp;
  rclcpp::Time pcl_time =
      this->now() - rclcpp::Duration::from_seconds(0.008);  // approximate time delay

  // Reset lidar off timer
  this->lidar_off_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(2000), std::bind(&Perception::lidar_timer_callback, this));

  rclcpp::Time start_time = this->now();
  rclcpp::Time time1, time2, time3, time4, time5, time6;

  // FOV Trimming
  auto trimmed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  _fov_trim_map_->at(_mission_type_)->fov_trimming(msg, trimmed_cloud);

  time1 = this->now();

  // Ground Removal
  auto ground_removed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  _ground_removal_->ground_removal(trimmed_cloud, ground_removed_cloud, *this->_ground_grid_);

  // Publish ground removed cloud for visualization
  ground_removed_cloud->header.frame_id = _vehicle_frame_id_;
  this->_ground_removed_publisher_->publish(*ground_removed_cloud);

  time2 = this->now();

  // Wall Removal
  auto wall_removed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  _wall_removal_->remove_walls(ground_removed_cloud, wall_removed_cloud);

  // Publish wall removed cloud for visualization
  wall_removed_cloud->header.frame_id = _vehicle_frame_id_;
  this->_wall_removed_publisher_->publish(*wall_removed_cloud);

  time3 = this->now();

  // Deskewing
  this->_deskew_->deskew_point_cloud(wall_removed_cloud, this->_vehicle_velocity_);

  time4 = this->now();

  // Clustering
  std::vector<Cluster> clusters;
  _clustering_->clustering(wall_removed_cloud, &clusters);

  time5 = this->now();

  // Evaluator
  std::vector<Cluster> filtered_clusters;

  for (auto& cluster : clusters) {
    if (_cone_evaluator_->evaluateCluster(cluster, *this->_ground_grid_)) {
      filtered_clusters.push_back(cluster);
    }
  }

  time6 = this->now();

  double perception_execution_time_seconds = (time6 - start_time).seconds();
  this->_execution_times_->at(0) = perception_execution_time_seconds * 1000.0;
  this->_execution_times_->at(1) = (time1 - start_time).seconds() * 1000.0;
  this->_execution_times_->at(2) = (time2 - time1).seconds() * 1000.0;
  this->_execution_times_->at(3) = (time3 - time2).seconds() * 1000.0;
  this->_execution_times_->at(4) = (time4 - time3).seconds() * 1000.0;
  this->_execution_times_->at(5) = (time5 - time4).seconds() * 1000.0;
  std_msgs::msg::Float64MultiArray exec_time_msg;
  exec_time_msg.data = *(this->_execution_times_);
  this->_perception_execution_time_publisher_->publish(exec_time_msg);

  publish_cones(&filtered_clusters, perception_execution_time_seconds, pcl_time);
}

void Perception::publish_cones(std::vector<Cluster>* cones, double exec_time,
                               rclcpp::Time pcl_time) {
  auto message = custom_interfaces::msg::PerceptionOutput();
  std::vector<custom_interfaces::msg::Cone> message_array = {};
  message.header = ::header;
  message.header.stamp = pcl_time;
  for (int i = 0; i < static_cast<int>(cones->size()); i++) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = cones->at(i).get_centroid().x();
    position.y = cones->at(i).get_centroid().y();

    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = cones->at(i).get_color();
    cone_message.is_large = cones->at(i).get_is_large();
    cone_message.confidence = cones->at(i).get_confidence();
    message.cones.cone_array.push_back(cone_message);
    message_array.push_back(cone_message);
  }
  message.exec_time = exec_time;

  this->_cones_publisher->publish(message);
  // TODO: correct frame id to LiDAR instead of vehicle
  this->_cone_marker_array_->publish(common_lib::communication::marker_array_from_structure_array(
      message_array, "perception", this->_vehicle_frame_id_, "green"));
}

void Perception::velocities_callback(const custom_interfaces::msg::Velocities& msg) {
  this->_vehicle_velocity_ =
      common_lib::structures::Velocities(msg.velocity_x, msg.velocity_y, msg.angular_velocity);
}

void Perception::lidar_timer_callback() {
  emergency_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (future.get()->success) {
          RCLCPP_WARN(this->get_logger(), "Emergency signal sent");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to send emergency signal");
        }
      });
  return;
}