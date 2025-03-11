#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <condition_variable>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <limits>
#include <mutex>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rosgraph_msgs/msg/clock.h"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "VehicleModel/VehicleModelBicycle.hpp"

#include "configParser.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "logger.hpp"
#include "pacsim/msg/perception_detections.hpp"
#include "pacsim/msg/stamped_scalar.hpp"
#include "pacsim/msg/wheels.hpp"
#include "pacsim/srv/clock_trigger_absolute.hpp"
#include "pacsim/srv/clock_trigger_relative.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reportWriter.hpp"
#include "ros2Helpers.hpp"
#include "rosgraph_msgs/msg/clock.h"
#include "sensorModels/gnssSensor.hpp"
#include "sensorModels/imuSensor.hpp"
#include "sensorModels/perceptionSensor.hpp"
#include "sensorModels/scalarValueSensor.hpp"
#include "sensorModels/wheelsSensor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "track/gripMap.hpp"
#include "track/trackLoader.hpp"
#include "types.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Constants
constexpr double DEFAULT_TIMESTEP = 1.0 / 1000.0;
constexpr double DEFAULT_EGO_MOTION_SENSOR_RATE = 200.0;
constexpr double DEFAULT_STEERING_DEADTIME = 0.05;
constexpr double DEFAULT_THROTTLE_DEADTIME = 0.02;
constexpr double DEFAULT_TORQUES_DEADTIME = 0.02;
constexpr double DEFAULT_POWER_GROUND_DEADTIME = 0.05;

/**
 * @brief Main simulation class that encapsulates simulation state and functionality
 */
class PacSimNode {
public:
  PacSimNode(std::shared_ptr<rclcpp::Node> node)
      : node_(node), logger_(std::make_shared<Logger>()) {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  }

  ~PacSimNode() = default;

  bool initialize() {
    if (!getRos2Params()) {
      logger_->logError("Failed to get ROS2 parameters");
      return false;
    }

    if (!fillMainConfig()) {
      logger_->logError("Failed to initialize main configuration");
      return false;
    }

    if (!initPerceptionSensors()) {
      logger_->logError("Failed to initialize perception sensors");
      return false;
    }

    if (!initSensors()) {
      logger_->logError("Failed to initialize sensors");
      return false;
    }

    if (!initVehicleModel()) {
      logger_->logError("Failed to initialize vehicle model");
      return false;
    }

    setupPublishers();
    setupSubscribers();
    setupServices();
    handleTf2StaticTransforms();

    logger_->logInfo("PacSimNode initialized successfully");
    return true;
  }

  int run() {
    std::unique_ptr<tf2_ros::TransformBroadcaster> br =
        std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // Load track and setup model
    Eigen::Vector3d start_position;
    Eigen::Vector3d start_orientation;
    Track track_landmarks;

    try {
      track_landmarks = loadMap(track_name_, start_position, start_orientation);
    } catch (const std::exception& e) {
      logger_->logError("Failed to load track: " + std::string(e.what()));
      return 1;
    }

    model_->setPosition(start_position);
    model_->setOrientation(start_orientation);

    // Setup grip map
    gripMap grip_map(logger_);
    grip_map.loadConfig(grip_map_path_);

    // Handle pre-transform if needed
    if (main_config_.pre_transform_track) {
      track_landmarks = transformTrack(track_landmarks, start_position, start_orientation);
      model_->setPosition(Eigen::Vector3d::Zero());
      model_->setOrientation(Eigen::Vector3d::Zero());
      grip_map.transformPoints(start_position, start_orientation);
      start_position = Eigen::Vector3d::Zero();
      start_orientation = Eigen::Vector3d::Zero();
    }

    LandmarkList track_as_lm_list = trackToLMList(track_landmarks);

    // Setup visualization
    LandmarksMarkerWrapper map_markers_wrapper(0.8, "pacsim");
    visualization_msgs::msg::MarkerArray map_marker_msg =
        map_markers_wrapper.markerFromLMs(track_landmarks, track_frame_, 0.0);
    map_viz_pub_->publish(map_marker_msg);

    // Setup competition logic
    competition_logic_ = std::make_shared<CompetitionLogic>(logger_, track_landmarks, main_config_);

    // Setup simulation loop timing
    auto next_loop_time = std::chrono::steady_clock::now();
    double last_ego_motion_sensor_sample_time = 0.0;
    bool finish = false;

    // Main loop with proper locking
    while (rclcpp::ok() && !finish) {
      publishClockMessage();

      // Integrate vehicle model forward
      auto wheel_positions = model_->getWheelPositions();
      Wheels grip_values = grip_map.getGripValues(wheel_positions);
      model_->forwardIntegrate(DEFAULT_TIMESTEP);

      // Get current state
      auto position = model_->getPosition();
      auto euler_angles = model_->getOrientation();
      auto angular_acceleration = model_->getAngularAcceleration();

      // Check competition status
      finish =
          competition_logic_->performAllChecks(track_landmarks, sim_time_, position, euler_angles);

      // Handle transforms and sensor updates
      publishTransforms(position, euler_angles, br);
      processDeadTimes();
      updateSensors(position, euler_angles, angular_acceleration, track_landmarks, track_as_lm_list,
                    grip_map);

      // Update simulation time with proper locking
      {
        std::lock_guard<std::mutex> lock(mutex_sim_time_);
        sim_time_ += DEFAULT_TIMESTEP;
      }

      // Handle clock trigger
      if (sim_time_ >= clock_stop_time_) {
        std::unique_lock<std::mutex> lock(mutex_clock_trigger_);
        cv_clock_trigger_.wait(lock);
        next_loop_time = std::chrono::steady_clock::now();
      }

      // Sleep to maintain desired simulation rate
      next_loop_time += std::chrono::microseconds(
          static_cast<int>((DEFAULT_TIMESTEP / realtime_ratio_) * 1000000.0));
      std::this_thread::sleep_until(next_loop_time);
    }

    // Generate final report
    logger_->logWarning("Simulation finished, generating report");
    Report report;
    competition_logic_->fillReport(report, sim_time_);
    report.track_name = track_name_;
    reportToFile(report, report_file_dir_);

    return 0;
  }

  // Service callbacks with proper thread safety
  void handleFinishSignal(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    competition_logic_->setFinish(true);
    logger_->logInfo("Finish signal received");
  }

  void handleClockTriggerAbsolute(
      const std::shared_ptr<pacsim::srv::ClockTriggerAbsolute::Request> request,
      std::shared_ptr<pacsim::srv::ClockTriggerAbsolute::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    clock_stop_time_ = rclcpp::Time(request->stop_time).seconds();
    response->already_past = (clock_stop_time_ < sim_time_);
    cv_clock_trigger_.notify_all();
    logger_->logDebug("Clock trigger absolute: " + std::to_string(clock_stop_time_));
  }

  void handleClockTriggerRelative(
      const std::shared_ptr<pacsim::srv::ClockTriggerRelative::Request> request,
      std::shared_ptr<pacsim::srv::ClockTriggerRelative::Response> response) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    clock_stop_time_ = sim_time_ + rclcpp::Duration(request->runtime).seconds();
    response->stop_time = rclcpp::Time(static_cast<uint64_t>(clock_stop_time_ * 1e9));
    cv_clock_trigger_.notify_all();
    logger_->logDebug("Clock trigger relative: " + std::to_string(clock_stop_time_));
  }

  // Subscriber callbacks with proper thread safety
  void handleLateralControl(const pacsim::msg::StampedScalar& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    dead_time_steering_front_.addVal(msg.value, sim_time_);
    dead_time_steering_rear_.addVal(0.0, sim_time_);
  }

  void handleLongitudinalControl(const pacsim::msg::StampedScalar& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    dead_time_throttle_.addVal(msg.value, sim_time_);
  }

  void handleTorquesMin(const pacsim::msg::Wheels& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    Wheels min{msg.fl, msg.fr, msg.rl, msg.rr};
    dead_time_min_torques_.addVal(min, sim_time_);
  }

  void handleTorquesMax(const pacsim::msg::Wheels& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    Wheels max{msg.fl, msg.fr, msg.rl, msg.rr};
    dead_time_max_torques_.addVal(max, sim_time_);
  }

  void handleWheelspeeds(const pacsim::msg::Wheels& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    Wheels w{msg.fl, msg.fr, msg.rl, msg.rr};
    dead_time_wspd_setpoints_.addVal(w, sim_time_);
  }

  void handlePowerGround(const pacsim::msg::StampedScalar& msg) {
    std::lock_guard<std::mutex> lock(mutex_sim_time_);
    dead_time_power_ground_setpoint_.addVal(msg.value, sim_time_);
  }

private:
  // Node and resources
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<IVehicleModel> model_;
  std::shared_ptr<Logger> logger_;

  // Configuration
  double sim_time_ = 0.0;
  std::string track_name_;
  std::string grip_map_path_;
  std::string track_frame_;
  std::string report_file_dir_;
  std::string main_config_path_;
  std::string perception_config_path_;
  std::string sensors_config_path_;
  std::string vehicle_model_config_path_;
  std::string discipline_;
  double realtime_ratio_ = 1.0;
  MainConfig main_config_;
  std::vector<std::string> joint_names_ = {"FL_steer",  "FL_rotate", "FR_steer", "FR_rotate",
                                           "RR_rotate", "RL_rotate", "steering"};

  // Sensors
  std::vector<std::shared_ptr<PerceptionSensor>> perception_sensors_;
  std::vector<std::shared_ptr<ImuSensor>> imus_;
  std::vector<std::shared_ptr<GnssSensor>> gnss_sensors_;
  std::shared_ptr<ScalarValueSensor> steering_sensor_front_;
  std::shared_ptr<ScalarValueSensor> steering_sensor_rear_;
  std::shared_ptr<WheelsSensor> wheelspeed_sensor_;
  std::shared_ptr<WheelsSensor> torques_sensor_;
  std::shared_ptr<ScalarValueSensor> current_sensor_ts_;
  std::shared_ptr<ScalarValueSensor> voltage_sensor_ts_;

  // Publishers
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_viz_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_front_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr steering_rear_pub_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr wheelspeed_pub_;
  rclcpp::Publisher<pacsim::msg::Wheels>::SharedPtr torques_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr voltage_sensor_ts_pub_;
  rclcpp::Publisher<pacsim::msg::StampedScalar>::SharedPtr current_sensor_ts_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  // Maps for sensor publishers
  std::vector<rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr> lms_pubs_;
  std::map<std::shared_ptr<PerceptionSensor>,
           rclcpp::Publisher<pacsim::msg::PerceptionDetections>::SharedPtr>
      perception_sensor_publisher_map_;
  std::map<std::shared_ptr<PerceptionSensor>, std::shared_ptr<LandmarksMarkerWrapper>>
      perception_sensor_markers_wrappers_map_;
  std::map<std::shared_ptr<PerceptionSensor>,
           rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr>
      perception_sensor_viz_publisher_map_;
  std::map<std::shared_ptr<ImuSensor>, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr>
      imu_publisher_map_;
  std::map<std::shared_ptr<GnssSensor>, rclcpp::Publisher<pacsim::msg::GNSS>::SharedPtr>
      gnss_publisher_map_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Dead time components
  DeadTime<double> dead_time_steering_front_{DEFAULT_STEERING_DEADTIME};
  DeadTime<double> dead_time_steering_rear_{DEFAULT_STEERING_DEADTIME};
  DeadTime<double> dead_time_throttle_{DEFAULT_THROTTLE_DEADTIME};
  DeadTime<Wheels> dead_time_torques_{DEFAULT_TORQUES_DEADTIME};
  DeadTime<Wheels> dead_time_wspd_setpoints_{DEFAULT_TORQUES_DEADTIME};
  DeadTime<Wheels> dead_time_max_torques_{DEFAULT_TORQUES_DEADTIME};
  DeadTime<Wheels> dead_time_min_torques_{DEFAULT_TORQUES_DEADTIME};
  DeadTime<double> dead_time_power_ground_setpoint_{DEFAULT_POWER_GROUND_DEADTIME};

  // Synchronization primitives
  std::mutex mutex_sim_time_;
  std::mutex mutex_clock_trigger_;
  std::condition_variable cv_clock_trigger_;
  double clock_stop_time_ = std::numeric_limits<double>::max();

  // Competition logic
  std::shared_ptr<CompetitionLogic> competition_logic_;

  // Helper methods
  bool getRos2Params() {
    std::vector<std::pair<std::string, std::string*>> params;
    params.push_back({"track_name", &track_name_});
    params.push_back({"grip_map_path", &grip_map_path_});
    params.push_back({"track_frame", &track_frame_});
    params.push_back({"report_file_dir", &report_file_dir_});
    params.push_back({"main_config_path", &main_config_path_});
    params.push_back({"perception_config_path", &perception_config_path_});
    params.push_back({"sensors_config_path", &sensors_config_path_});
    params.push_back({"vehicle_model_config_path", &vehicle_model_config_path_});
    params.push_back({"discipline", &discipline_});

    for (auto p : params) {
      node_->declare_parameter(std::string(p.first), rclcpp::PARAMETER_STRING);
      (*p.second) = node_->get_parameter(std::string(p.first)).as_string();
    }

    node_->declare_parameter("realtime_ratio", rclcpp::PARAMETER_DOUBLE);
    node_->get_parameter("realtime_ratio", realtime_ratio_);

    return true;
  }

  bool fillMainConfig() {
    try {
      Config cfg(main_config_path_);
      ConfigElement config = cfg.getElement("pacsim");
      config["timeouts"].getElement<double>(&main_config_.timeout_start, "start");
      config["timeouts"].getElement<double>(&main_config_.timeout_acceleration, "acceleration");
      config["timeouts"].getElement<double>(&main_config_.timeout_autocross, "autocross");
      config["timeouts"].getElement<double>(&main_config_.timeout_skidpad, "skidpad");
      config["timeouts"].getElement<double>(&main_config_.timeout_trackdrive_first,
                                            "trackdrive_first");
      config["timeouts"].getElement<double>(&main_config_.timeout_trackdrive_total,
                                            "trackdrive_total");

      config.getElement<std::string>(&main_config_.cog_frame_id_pipeline, "cog_frame_id_pipeline");
      config.getElement<bool>(&main_config_.broadcast_sensors_tf2, "broadcast_sensors_tf2");

      config.getElement<bool>(&main_config_.oc_detect, "oc_detect");
      config.getElement<bool>(&main_config_.doo_detect, "doo_detect");
      config.getElement<bool>(&main_config_.uss_detect, "uss_detect");
      config.getElement<bool>(&main_config_.finish_validate, "finish_validate");

      config.getElement<bool>(&main_config_.pre_transform_track, "pre_transform_track");

      main_config_.discipline = stringToDiscipline(discipline_);
      return true;
    } catch (const std::exception& e) {
      logger_->logError("Error loading main config: " + std::string(e.what()));
      return false;
    }
  }

  bool initPerceptionSensors() {
    try {
      Config cfg(perception_config_path_);
      auto perception_sensors_config = cfg.getElement("perception_sensors");
      std::vector<ConfigElement> sensors;
      perception_sensors_config.getElements(&sensors);

      for (auto& sensor : sensors) {
        std::shared_ptr<PerceptionSensor> perception_sensor = std::make_shared<PerceptionSensor>();
        perception_sensor->readConfig(sensor);
        perception_sensors_.push_back(perception_sensor);
      }
      return true;
    } catch (const std::exception& e) {
      logger_->logError("Error initializing perception sensors: " + std::string(e.what()));
      return false;
    }
  }

  bool initSensors() {
    try {
      Config cfg(sensors_config_path_);
      auto sensors_config = cfg.getElement("sensors");

      auto gnss_configs = sensors_config.getElement("gnssSensors");
      std::vector<ConfigElement> gnss_sensor_configs;
      gnss_configs.getElements(&gnss_sensor_configs);
      for (auto& sensor : gnss_sensor_configs) {
        std::shared_ptr<GnssSensor> gnss_sensor = std::make_shared<GnssSensor>();
        gnss_sensor->readConfig(sensor);
        gnss_sensors_.push_back(gnss_sensor);
      }

      auto imu_configs = sensors_config.getElement("imus");
      std::vector<ConfigElement> sensors;
      imu_configs.getElements(&sensors);
      for (auto& sensor : sensors) {
        std::shared_ptr<ImuSensor> imu_sensor = std::make_shared<ImuSensor>(200.0, 0.002);
        imu_sensor->readConfig(sensor);
        imus_.push_back(imu_sensor);
      }

      steering_sensor_front_ = std::make_shared<ScalarValueSensor>(200.0, 0.005);
      auto front_steering_config = sensors_config.getElement("steering_front");
      steering_sensor_front_->readConfig(front_steering_config);

      steering_sensor_rear_ = std::make_shared<ScalarValueSensor>(200.0, 0.005);
      auto rear_steering_config = sensors_config.getElement("steering_front");
      steering_sensor_rear_->readConfig(rear_steering_config);

      auto wheel_speed_config = sensors_config.getElement("wheelspeeds");
      wheelspeed_sensor_ = std::make_shared<WheelsSensor>(200.0, 0.005);
      wheelspeed_sensor_->readConfig(wheel_speed_config);

      voltage_sensor_ts_ = std::make_shared<ScalarValueSensor>(200.0, 0.005);
      auto voltage_ts_config = sensors_config.getElement("voltage_ts");
      voltage_sensor_ts_->readConfig(voltage_ts_config);

      current_sensor_ts_ = std::make_shared<ScalarValueSensor>(200.0, 0.005);
      auto current_ts_config = sensors_config.getElement("current_ts");
      current_sensor_ts_->readConfig(current_ts_config);

      auto torques_config = sensors_config.getElement("wheelspeeds");
      torques_sensor_ = std::make_shared<WheelsSensor>(200.0, 0.005);
      torques_sensor_->readConfig(torques_config);

      return true;
    } catch (const std::exception& e) {
      logger_->logError("Error initializing sensors: " + std::string(e.what()));
      return false;
    }
  }

  bool initVehicleModel() {
    try {
      model_ = std::make_shared<VehicleModelBicycle>();
      Config model_config(vehicle_model_config_path_);
      auto config_vehicle_model = model_config.getElement("vehicle_model");
      model_->readConfig(config_vehicle_model);
      return true;
    } catch (const std::exception& e) {
      logger_->logError("Error initializing vehicle model: " + std::string(e.what()));
      return false;
    }
  }

  void setupPublishers() {
    // Setup main publishers
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
    velocity_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/pacsim/velocity", 3);
    map_viz_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/pacsim/map", 1);

    // Setup sensor publishers
    steering_front_pub_ =
        node_->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringFront", 1);
    steering_rear_pub_ =
        node_->create_publisher<pacsim::msg::StampedScalar>("/pacsim/steeringRear", 1);
    voltage_sensor_ts_pub_ =
        node_->create_publisher<pacsim::msg::StampedScalar>("/pacsim/ts/voltage", 1);
    current_sensor_ts_pub_ =
        node_->create_publisher<pacsim::msg::StampedScalar>("/pacsim/ts/current", 1);
    wheelspeed_pub_ = node_->create_publisher<pacsim::msg::Wheels>("/pacsim/wheelspeeds", 1);
    torques_pub_ = node_->create_publisher<pacsim::msg::Wheels>("/pacsim/torques", 1);
    joint_state_publisher_ =
        node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 3);

    for (auto& sensor : perception_sensors_) {
      auto detections_markers_wrapper =
          std::make_shared<LandmarksMarkerWrapper>(0.8, "pacsim/" + sensor->getName());

      auto pub = node_->create_publisher<pacsim::msg::PerceptionDetections>(
          "/pacsim/perception/" + sensor->getName() + "/landmarks", 1);

      auto pub_viz = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/pacsim/perception/" + sensor->getName() + "/visualization", 1);

      lms_pubs_.push_back(pub);
      perception_sensor_publisher_map_[sensor] = pub;
      perception_sensor_markers_wrappers_map_[sensor] = detections_markers_wrapper;
      perception_sensor_viz_publisher_map_[sensor] = pub_viz;

      pub_viz->publish(detections_markers_wrapper->deleteAllMsg(sensor->getFrameId()));
    }

    for (auto& imu : imus_) {
      auto pub = node_->create_publisher<sensor_msgs::msg::Imu>("/pacsim/imu/" + imu->getName(), 3);
      imu_publisher_map_[imu] = pub;
    }

    for (auto& gnss : gnss_sensors_) {
      auto pub = node_->create_publisher<pacsim::msg::GNSS>("/pacsim/gnss/" + gnss->getName(), 3);
      gnss_publisher_map_[gnss] = pub;
    }
  }

  void setupSubscribers() {
    subscriptions_.push_back(node_->create_subscription<pacsim::msg::StampedScalar>(
        "/pacsim/steering_setpoint", 1,
        std::bind(&PacSimNode::handleLateralControl, this, std::placeholders::_1)));

    subscriptions_.push_back(node_->create_subscription<pacsim::msg::StampedScalar>(
        "/pacsim/throttle_setpoint", 1,
        std::bind(&PacSimNode::handleLongitudinalControl, this, std::placeholders::_1)));

    subscriptions_.push_back(node_->create_subscription<pacsim::msg::Wheels>(
        "/pacsim/torques_min", 1,
        std::bind(&PacSimNode::handleTorquesMin, this, std::placeholders::_1)));

    subscriptions_.push_back(node_->create_subscription<pacsim::msg::Wheels>(
        "/pacsim/torques_max", 1,
        std::bind(&PacSimNode::handleTorquesMax, this, std::placeholders::_1)));

    subscriptions_.push_back(node_->create_subscription<pacsim::msg::Wheels>(
        "/pacsim/wspd_setpoints", 1,
        std::bind(&PacSimNode::handleWheelspeeds, this, std::placeholders::_1)));

    subscriptions_.push_back(node_->create_subscription<pacsim::msg::StampedScalar>(
        "/pacsim/power_ground_setpoint", 1,
        std::bind(&PacSimNode::handlePowerGround, this, std::placeholders::_1)));
  }

  void setupServices() {
    node_->create_service<std_srvs::srv::Empty>(
        "/pacsim/finish_signal", std::bind(&PacSimNode::handleFinishSignal, this,
                                           std::placeholders::_1, std::placeholders::_2));

    node_->create_service<pacsim::srv::ClockTriggerAbsolute>(
        "/pacsim/clock_trigger/absolute", std::bind(&PacSimNode::handleClockTriggerAbsolute, this,
                                                    std::placeholders::_1, std::placeholders::_2));

    node_->create_service<pacsim::srv::ClockTriggerRelative>(
        "/pacsim/clock_trigger/relative", std::bind(&PacSimNode::handleClockTriggerRelative, this,
                                                    std::placeholders::_1, std::placeholders::_2));
  }

  void handleTf2StaticTransforms() {
    if (main_config_.broadcast_sensors_tf2) {
      for (auto& sensor : perception_sensors_) {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = rclcpp::Time(0, 0);
        t.header.frame_id = main_config_.cog_frame_id_pipeline;
        t.child_frame_id = sensor->getFrameId();

        auto translation = sensor->getPosition();
        auto orientation = sensor->getOrientation();

        t.transform.translation.x = translation.x();
        t.transform.translation.y = translation.y();
        t.transform.translation.z = translation.z();

        tf2::Quaternion q;
        q.setRPY(orientation.x(), orientation.y(), orientation.z());
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
      }
    }
  }

  void publishClockMessage() {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = rclcpp::Time(static_cast<uint64_t>(sim_time_ * 1e9));
    clock_pub_->publish(clock_msg);
  }

  void publishTransforms(const Eigen::Vector3d& position, const Eigen::Vector3d& euler_angles,
                         std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster) {
    geometry_msgs::msg::TransformStamped transform_stamped =
        createRosTransformMsg(position, euler_angles, track_frame_, "car", sim_time_);
    broadcaster->sendTransform(transform_stamped);
  }

  void processDeadTimes() {
    if (dead_time_steering_front_.availableDeadTime(sim_time_)) {
      double val = dead_time_steering_front_.getOldest();
      model_->setSteeringSetpointFront(val);
    }

    if (dead_time_steering_rear_.availableDeadTime(sim_time_)) {
      double val = dead_time_steering_rear_.getOldest();
      model_->setSteeringSetpointRear(val);
    }

    if (dead_time_torques_.availableDeadTime(sim_time_)) {
      Wheels val = dead_time_torques_.getOldest();
      model_->setTorques(val);
    }

    if (dead_time_wspd_setpoints_.availableDeadTime(sim_time_)) {
      Wheels val = dead_time_wspd_setpoints_.getOldest();
      model_->setRpmSetpoints(val);
    }

    if (dead_time_max_torques_.availableDeadTime(sim_time_)) {
      Wheels val = dead_time_max_torques_.getOldest();
      model_->setMaxTorques(val);
    }

    if (dead_time_min_torques_.availableDeadTime(sim_time_)) {
      Wheels val = dead_time_min_torques_.getOldest();
      model_->setMinTorques(val);
    }

    if (dead_time_throttle_.availableDeadTime(sim_time_)) {
      double val = dead_time_throttle_.getOldest();
      model_->setThrottle(val);
    }

    if (dead_time_power_ground_setpoint_.availableDeadTime(sim_time_)) {
      double val = dead_time_power_ground_setpoint_.getOldest();
      model_->setPowerGroundSetpoint(val);
    }
  }

  void updateSensors(Eigen::Vector3d& position, Eigen::Vector3d& euler_angles,
                     Eigen::Vector3d& angular_acceleration, Track& track_landmarks,
                     LandmarkList& track_as_lm_list, gripMap& grip_map) {
    Eigen::Vector3d vel = model_->getVelocity();
    Eigen::Vector3d rot = model_->getAngularVelocity();
    geometry_msgs::msg::TwistWithCovarianceStamped vel_msg =
        createRosTwistMsg(vel, rot, "car", sim_time_);
    velocity_pub_->publish(vel_msg);

    ImuData imu_data_cog{model_->getAcceleration(),
                         model_->getAngularVelocity(),
                         Eigen::Matrix3d::Zero(),
                         Eigen::Matrix3d::Zero(),
                         sim_time_,
                         ""};

    for (auto& imu : imus_) {
      if (imu->RunTick(imu_data_cog, angular_acceleration, sim_time_)) {
        ImuData imu_data = imu->getOldest();
        sensor_msgs::msg::Imu imu_msg = createRosImuMsg(imu_data);
        imu_publisher_map_[imu]->publish(imu_msg);
      }
    }

    Wheels steering_curr = model_->getSteeringAngles();
    double steering_wheel_curr = model_->getSteeringWheelAngle();

    StampedScalar steering_data_front{steering_wheel_curr, sim_time_};
    if (steering_sensor_front_->RunTick(steering_data_front, sim_time_)) {
      StampedScalar steering_data = steering_sensor_front_->getOldest();
      pacsim::msg::StampedScalar msg;
      msg.value = steering_data.data;
      msg.stamp = rclcpp::Time(static_cast<uint64_t>(steering_data.timestamp * 1e9));
      steering_front_pub_->publish(msg);
    }

    StampedScalar steering_data_rear{(steering_curr.RL + steering_curr.RR) * 0.5, sim_time_};
    if (steering_sensor_rear_->RunTick(steering_data_rear, sim_time_)) {
      StampedScalar steering_data = steering_sensor_rear_->getOldest();
      pacsim::msg::StampedScalar msg;
      msg.value = steering_data.data;
      msg.stamp = rclcpp::Time(static_cast<uint64_t>(steering_data.timestamp * 1e9));
      steering_rear_pub_->publish(msg);
    }

    // Update voltage/current TS sensors
    double voltage_ts_curr = model_->getVoltageTS();
    StampedScalar voltage_ts_data{voltage_ts_curr, sim_time_};
    if (voltage_sensor_ts_->RunTick(voltage_ts_data, sim_time_)) {
      StampedScalar voltage_data = voltage_sensor_ts_->getOldest();
      pacsim::msg::StampedScalar msg;
      msg.value = voltage_data.data;
      msg.stamp = rclcpp::Time(static_cast<uint64_t>(voltage_data.timestamp * 1e9));
      voltage_sensor_ts_pub_->publish(msg);
    }

    double current_ts_curr = model_->getCurrentTS();
    StampedScalar current_ts_data{current_ts_curr, sim_time_};
    if (current_sensor_ts_->RunTick(current_ts_data, sim_time_)) {
      StampedScalar current_data = current_sensor_ts_->getOldest();
      pacsim::msg::StampedScalar msg;
      msg.value = current_data.data;
      msg.stamp = rclcpp::Time(static_cast<uint64_t>(current_data.timestamp * 1e9));
      current_sensor_ts_pub_->publish(msg);
    }

    for (auto& gnss : gnss_sensors_) {
      if (gnss->RunTick(track_landmarks.gnssOrigin, track_landmarks.enuToTrackRotation, position,
                        euler_angles, sim_time_, model_->getVelocity(),
                        model_->getAngularVelocity(), Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(), main_config_.pre_transform_track)) {
        auto gnss_data = gnss->getOldest();
        auto gnss_msg = createRosGnssMessage(gnss_data);
        gnss_publisher_map_[gnss]->publish(gnss_msg);
      }
    }

    for (auto& perception_sensor : perception_sensors_) {
      if (perception_sensor->RunTick(track_as_lm_list, position, euler_angles, sim_time_)) {
        LandmarkList sensor_lms = perception_sensor->getOldest();
        // Convert back to track format for visualization
        Track track_msg = lmListToTrack(sensor_lms);
        visualization_msgs::msg::MarkerArray lms_marker_msg =
            perception_sensor_markers_wrappers_map_[perception_sensor]->markerFromLMs(
                track_msg, sensor_lms.frame_id, sensor_lms.timestamp);
        perception_sensor_viz_publisher_map_[perception_sensor]->publish(lms_marker_msg);

        pacsim::msg::PerceptionDetections lms_msg =
            LandmarkListToRosMessage(sensor_lms, sensor_lms.frame_id, sensor_lms.timestamp);
        perception_sensor_publisher_map_[perception_sensor]->publish(lms_msg);
      }
    }

    Wheels wheelspeed = model_->getWheelspeeds();
    Wheels torques = model_->getTorques();
    Wheels wheel_orientations = model_->getWheelOrientations();

    wheelspeed.timestamp = sim_time_;
    torques.timestamp = sim_time_;

    if (wheelspeed_sensor_->RunTick(wheelspeed, position, euler_angles, sim_time_)) {
      Wheels wspd = wheelspeed_sensor_->getOldest();
      pacsim::msg::Wheels wspd_msg;
      wspd_msg.fl = wspd.FL;
      wspd_msg.fr = wspd.FR;
      wspd_msg.rl = wspd.RL;
      wspd_msg.rr = wspd.RR;
      wspd_msg.stamp = rclcpp::Time(static_cast<uint64_t>(wspd.timestamp * 1e9));
      wheelspeed_pub_->publish(wspd_msg);
    }

    if (torques_sensor_->RunTick(torques, position, euler_angles, sim_time_)) {
      Wheels trq = torques_sensor_->getOldest();
      pacsim::msg::Wheels torques_msg;
      torques_msg.fl = trq.FL;
      torques_msg.fr = trq.FR;
      torques_msg.rl = trq.RL;
      torques_msg.rr = trq.RR;
      torques_msg.stamp = rclcpp::Time(static_cast<uint64_t>(trq.timestamp * 1e9));
      torques_pub_->publish(torques_msg);
    }

    std::vector<double> joint_msg = {
        steering_curr.FL,      wheel_orientations.FL, steering_curr.FR,    wheel_orientations.FR,
        wheel_orientations.RR, wheel_orientations.RL, -steering_wheel_curr};
    sensor_msgs::msg::JointState joint_stamped =
        createRosJointMsg(joint_names_, joint_msg, sim_time_);
    joint_state_publisher_->publish(joint_stamped);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pacsim_node");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto pacsim_node = std::make_shared<PacSimNode>(node);
  if (!pacsim_node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize PacSimNode");
    return 1;
  }

  executor->add_node(node);

  // Run simulation in a separate thread
  std::thread simulation_thread([&pacsim_node]() { return pacsim_node->run(); });

  RCLCPP_INFO(node->get_logger(), "Started pacsim");

  executor->spin();

  // Wait for simulation thread to finish
  simulation_thread.join();

  RCLCPP_INFO(node->get_logger(), "Finished pacsim");
  rclcpp::shutdown();

  return 0;
}