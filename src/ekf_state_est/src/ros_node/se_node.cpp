#include "ros_node/se_node.hpp"

#include <fstream>

#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"
#include "adapter_ekf_state_est/map.hpp"
#include "common_lib/communication/marker.hpp"
#include "common_lib/maths/transformations.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/config_load/config_load.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "motion_lib/bicycle_model.hpp"
#include "motion_lib/car_parameters.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
/*---------------------- Constructor --------------------*/

double last_wss = 0.0;

SENode::SENode() : Node("ekf_state_est") {

  auto global_config_path = common_lib::config_load::get_config_yaml_path("ekf_state_est", "global",
                                                                    "global_config");
  auto global_config = YAML::LoadFile(global_config_path);

  _use_simulated_perception_ = global_config["global"]["use_simulated_perception"].as<bool>();
  _adapter_name_ = global_config["global"]["adapter"].as<std::string>();

  auto se_config_path = common_lib::config_load::get_config_yaml_path("ekf_state_est", "ekf_state_est",
                                                                _adapter_name_);

  auto se_config = YAML::LoadFile(se_config_path)["ekf_state_est"];
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "SE config contents: %s", YAML::Dump(se_config).c_str());

  _use_odometry_ = se_config["use_odometry"].as<bool>();
  std::string motion_model_name = se_config["motion_model"].as<std::string>();
  std::string data_assocation_model_name = se_config["data_association_model"].as<std::string>();

  float wss_noise = 0.0f;
  float imu_noise = 0.0f;  // Declare the 'imu_noise' variable
  if (data_assocation_model_name == "max_likelihood") {
    wss_noise = se_config["wss_noise"].as<float>();
    imu_noise = se_config["imu_noise"].as<float>();
  }
  float data_association_limit_distance = se_config["data_association_limit_distance"].as<float>();
  float observation_noise = se_config["observation_noise"].as<float>();

  std::shared_ptr<MotionModel> motion_model_wss = motion_model_constructors.at(
      "normal_velocity_model")(MotionModel::create_process_noise_covariance_matrix(wss_noise));
  std::shared_ptr<MotionModel> motion_model_imu = motion_model_constructors.at(motion_model_name)(
      MotionModel::create_process_noise_covariance_matrix(imu_noise));

  std::shared_ptr<ObservationModel> observation_model = std::make_shared<ObservationModel>(
      ObservationModel::create_observation_noise_covariance_matrix(observation_noise));
  std::shared_ptr<DataAssociationModel> data_association_model =
      data_association_model_constructors.at(data_assocation_model_name)(
          data_association_limit_distance);
  _ekf_ = std::make_shared<ExtendedKalmanFilter>(observation_model, data_association_model);
  _ekf_->add_motion_model("wheel_speed_sensor", motion_model_wss);
  _ekf_->add_motion_model("imu", motion_model_imu);
  _perception_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _motion_update_ = std::make_shared<MotionUpdate>();
  _track_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _vehicle_state_ = std::make_shared<common_lib::structures::VehicleState>();
  _motion_update_->last_update = this->get_clock()->now();

  if (!_use_simulated_perception_) {
    this->_perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 1,
        std::bind(&SENode::_perception_subscription_callback, this, std::placeholders::_1));
  }
  this->_vehicle_state_publisher_ = this->create_publisher<custom_interfaces::msg::VehicleState>(
      "/state_estimation/vehicle_state", 10);
  this->_vehicle_state_publisher_wss_ =
      this->create_publisher<custom_interfaces::msg::VehicleState>(
          "/state_estimation/vehicle_state/wss", 10);
  this->_vehicle_state_publisher_imu_ =
      this->create_publisher<custom_interfaces::msg::VehicleState>(
          "/state_estimation/vehicle_state/imu", 10);
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
  this->_visualization_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map", 10);
  this->_correction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/correction_step", 10);
  this->_prediction_execution_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/state_estimation/execution_time/prediction_step", 10);
  _adapter_ = adapter_map.at(_adapter_name_)(std::shared_ptr<SENode>(this));

  this->_position_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/state_estimation/visualization/position", 10);
  this->_car_model_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/state_estimation/visualization/car_model", 10);
}

/*---------------------- Subscriptions --------------------*/

void SENode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;
  if (this->_perception_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
    return;
  }

  if (!this->_go_) {
    return;
  }

  rclcpp::Time start_time = this->get_clock()->now();

  this->_perception_map_->clear();

  for (auto &cone : cone_array) {
    this->_perception_map_->push_back(common_lib::structures::Cone(cone.position.x, cone.position.y,
                                                                   cone.color, cone.confidence));
  }

  if (this->_ekf_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  // RCLCPP_DEBUG(this->get_logger(), "CORRECTION STEP END\n---------------------------\n");
  this->_ekf_->correction_step(*(this->_perception_map_));
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);

  rclcpp::Time end_time = this->get_clock()->now();

  // Execution Time calculation
  std_msgs::msg::Float64 correction_execution_time;
  correction_execution_time.data = (end_time - start_time).seconds() * 1000.0;
  this->_correction_execution_time_publisher_->publish(correction_execution_time);
  // this->_publish_vehicle_state();
  this->_publish_map();
}

void SENode::_imu_subscription_callback(const sensor_msgs::msg::Imu &imu_msg) {
  if (this->_use_odometry_) {
    return;
  }

  if (!this->_go_) {
    return;
  }

  rclcpp::Time start_time = this->get_clock()->now();

  double ax = imu_msg.linear_acceleration.x;
  // double ay = imu_msg.linear_acceleration.y;

  double v_rot = imu_msg.angular_velocity.z;

  double angle = this->_ekf_->get_state()(2);

  double ax_map = ax * cos(angle) /* - ay * sin(angle)*/;
  double ay_map = ax * sin(angle) /* + ay * cos(angle) */;

  MotionUpdate motion_prediction_data;
  motion_prediction_data.acceleration_x = ax_map;
  motion_prediction_data.acceleration_y = ay_map;
  motion_prediction_data.rotational_velocity = v_rot;
  this->_motion_update_->acceleration_x = motion_prediction_data.acceleration_x;
  this->_motion_update_->acceleration_y = motion_prediction_data.acceleration_y;
  this->_motion_update_->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update_->last_update = imu_msg.header.stamp;
  if (this->_ekf_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);
  this->_ekf_->prediction_step(temp_update, "imu");
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);

  // Execution Time calculation
  rclcpp::Time end_time = this->get_clock()->now();
  std_msgs::msg::Float64 prediction_execution_time;
  prediction_execution_time.data = (end_time - start_time).seconds() * 1000.0;
  this->_prediction_execution_time_publisher_->publish(prediction_execution_time);

  this->_publish_vehicle_state_imu();
  this->_publish_map();
}

double rl_before = 0.0, rr_before = 0.0, fl_before = 0.0, fr_before = 0.0;
double difference = 10;

void SENode::_wheel_speeds_subscription_callback(double rl_speed, double rr_speed, double fl_speed,
                                                 double fr_speed, double steering_angle,
                                                 const rclcpp::Time &timestamp) {
  if (!this->_go_) {
    return;
  }

  bool change = false;

  if (abs(rl_before - rl_speed) >= difference) change = true;
  if (abs(rr_before - rr_speed) >= difference) change = true;
  if (abs(fl_before - fl_speed) >= difference) change = true;
  if (abs(fr_before - fr_speed) >= difference) change = true;

  rl_before = rl_speed;
  rr_before = rr_speed;
  fl_before = fl_speed;
  fr_before = fr_speed;
  if (change) return;

  RCLCPP_INFO(this->get_logger(), "Rear Left: %f\n Rear Right: %f", rl_speed, rr_speed);
  rclcpp::Time start_time = this->get_clock()->now();

  auto [linear_velocity, angular_velocity] =
      motion_lib::bicycle_model::odometry_to_velocities_transform(rl_speed, fl_speed, rr_speed,
                                                                  fr_speed, steering_angle);

  RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f\nAngular Velocity: %f", linear_velocity,
              angular_velocity);
  MotionUpdate motion_prediction_data;
  motion_prediction_data.translational_velocity = linear_velocity;
  motion_prediction_data.rotational_velocity = angular_velocity;
  this->_motion_update_->translational_velocity = motion_prediction_data.translational_velocity;
  this->_motion_update_->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update_->steering_angle = steering_angle;
  this->_motion_update_->last_update = timestamp;

  last_wss = linear_velocity;

  if (this->_ekf_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);

  RCLCPP_INFO(this->get_logger(), "Motion update Translational Velocity: %f",
              this->_motion_update_->translational_velocity);

  this->_ekf_->prediction_step(temp_update, "wheel_speed_sensor");
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);

  // Execution Time calculation
  rclcpp::Time end_time = this->get_clock()->now();
  std_msgs::msg::Float64 prediction_execution_time;
  prediction_execution_time.data = (end_time - start_time).seconds() * 1000.0;
  this->_prediction_execution_time_publisher_->publish(prediction_execution_time);

  this->_publish_vehicle_state_wss();
  this->_publish_map();
}
/*---------------------- Publications --------------------*/

void SENode::_publish_vehicle_state() {
  if (this->_vehicle_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  // if (!this->_go_){
  //   return;
  // }

  auto message = custom_interfaces::msg::VehicleState();
  message.position.x = this->_vehicle_state_->pose.position.x;
  message.position.y = this->_vehicle_state_->pose.position.y;
  message.theta = this->_vehicle_state_->pose.orientation;
  double velocity_x = this->_vehicle_state_->velocity_x;
  double velocity_y = this->_vehicle_state_->velocity_y;
  message.linear_velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  message.angular_velocity = this->_vehicle_state_->rotational_velocity;
  message.header.stamp = this->get_clock()->now();

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f); Velocities: (%f, %f)",
               message.position.x, message.position.y, message.theta, message.linear_velocity,
               message.angular_velocity);
  this->_vehicle_state_publisher_->publish(message);
}

void SENode::_publish_vehicle_state_wss() {
  if (this->_vehicle_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::VehicleState();
  message.position.x = this->_vehicle_state_->pose.position.x;
  message.position.y = this->_vehicle_state_->pose.position.y;
  message.theta = this->_vehicle_state_->pose.orientation;
  double velocity_x = this->_vehicle_state_->velocity_x;
  double velocity_y = this->_vehicle_state_->velocity_y;
  message.linear_velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  message.angular_velocity = this->_vehicle_state_->rotational_velocity;
  message.header.stamp = this->get_clock()->now();

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f); Velocities: (%f, %f)",
               message.position.x, message.position.y, message.theta, message.linear_velocity,
               message.angular_velocity);
  this->_vehicle_state_publisher_->publish(message);

  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";  // Use an appropriate frame
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "vehicle_state";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = this->_vehicle_state_->pose.position.x;
  marker.pose.position.y = this->_vehicle_state_->pose.position.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5; 
  marker.scale.z = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  RCLCPP_DEBUG(this->get_logger(), "PUB - Marker at position: (%f, %f)", marker.pose.position.x,
               marker.pose.position.y);
  this->_position_publisher_->publish(marker);

  // Calculate front and rear axle positions
  double lr = 0.791;
  double lf = 1.6;
  double theta = this->_vehicle_state_->pose.orientation;
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  geometry_msgs::msg::Point rear_axis;
  rear_axis.x = message.position.x - lr * cos_theta;
  rear_axis.y = message.position.y - lr * sin_theta;

  geometry_msgs::msg::Point front_axis;
  front_axis.x = message.position.x + lf * cos_theta;
  front_axis.y = message.position.y + lf * sin_theta;

  // Create a marker array to draw the line between front and rear axis
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = this->get_clock()->now();
  line_marker.ns = "vehicle_state";
  line_marker.id = 1;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.1;  // Line width
  line_marker.color.r = 1.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  line_marker.color.a = 1.0;

  // Add points to the line marker
  line_marker.points.push_back(rear_axis);
  line_marker.points.push_back(front_axis);

  // Publish the line marker
  this->_car_model_publisher_->publish(line_marker);
}

void SENode::_publish_vehicle_state_imu() {
  if (this->_vehicle_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::VehicleState();
  message.position.x = this->_vehicle_state_->pose.position.x;
  message.position.y = this->_vehicle_state_->pose.position.y;
  message.theta = this->_vehicle_state_->pose.orientation;
  double velocity_x = this->_vehicle_state_->velocity_x;
  double velocity_y = this->_vehicle_state_->velocity_y;
  message.linear_velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  message.angular_velocity = this->_vehicle_state_->rotational_velocity;
  message.header.stamp = this->get_clock()->now();

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f); Velocities: (%f, %f)",
               message.position.x, message.position.y, message.theta, message.linear_velocity,
               message.angular_velocity);
  this->_vehicle_state_publisher_->publish(message);
}

void SENode::_publish_map() {
  auto cone_array_msg = custom_interfaces::msg::ConeArray();
  auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (common_lib::structures::Cone const &cone : *this->_track_map_) {
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position.x = cone.position.x;
    cone_message.position.y = cone.position.y;
    cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
    cone_array_msg.cone_array.push_back(cone_message);
    RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
                 cone_message.position.y, cone_message.color.c_str());
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  cone_array_msg.header.stamp = this->get_clock()->now();
  marker_array_msg = common_lib::communication::marker_array_from_structure_array(
      *this->_track_map_, "map_cones", _adapter_name_ == "eufs" ? "base_footprint" : "map");
  this->_map_publisher_->publish(cone_array_msg);
  this->_visualization_map_publisher_->publish(marker_array_msg);
}

/*---------------------- Others --------------------*/

// Not utilized
void SENode::_ekf_step() {
  if (this->_ekf_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);
  this->_ekf_->prediction_step(temp_update, "wheel_speed_sensor");
  this->_ekf_->correction_step(*(this->_perception_map_));
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK Step");
}

void SENode::_update_and_publish() {  // Currently unused, as correction step
                                      // and prediction step are carried out
                                      // separately
  this->_ekf_step();
  this->_publish_vehicle_state();
  this->_publish_map();
}