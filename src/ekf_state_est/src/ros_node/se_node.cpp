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
#include "common_lib/vehicle_dynamics/bicycle_model.hpp"
#include "common_lib/vehicle_dynamics/car_parameters.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "visualization_msgs/msg/marker.hpp"
/*---------------------- Constructor --------------------*/

SENode::SENode() : Node("ekf_state_est") {
  // TODO: noise matrixes by parameter
  this->_use_odometry_ = this->declare_parameter("use_odometry", true);
  _use_simulated_perception_ = this->declare_parameter("use_simulated_perception", false);
  _adapter_name_ = this->declare_parameter("adapter", "eufs");
  std::string motion_model_name = this->declare_parameter("motion_model", "normal_velocity_model");
  std::string data_assocation_model_name =
      this->declare_parameter("data_assocation_model", "simple_ml");
  if (data_assocation_model_name == "simple_ml") {
    float sml_da_curvature = static_cast<float>(this->declare_parameter("sml_da_curvature", 15.0f));
    float sml_initial_limit =
        static_cast<float>(this->declare_parameter("sml_initial_limit", 0.1f));
    SimpleMaximumLikelihood::curvature_ = sml_da_curvature;
    SimpleMaximumLikelihood::initial_limit_ = sml_initial_limit;
  }
  // float observation_noise = static_cast<float>(this->declare_parameter("observation_noise",
  // 0.05f)); float wheel_speed_sensor_noise =
  //     static_cast<float>(this->declare_parameter("wheel_speed_sensor_noise", 0.003f));
  float data_association_limit_distance =
      static_cast<float>(this->declare_parameter("data_association_limit_distance", 71.0f));
  //
  //
  //
  //
  std::shared_ptr<MotionModel> motion_model_wss = motion_model_constructors.at(
      "normal_velocity_model")(MotionModel::create_process_noise_covariance_matrix(0.3f));  // TUNE
  std::shared_ptr<MotionModel> motion_model_imu = motion_model_constructors.at(motion_model_name)(
      MotionModel::create_process_noise_covariance_matrix(0.0064f));  // 0.0064//TUNE
  //
  //
  //
  std::shared_ptr<ObservationModel> observation_model = std::make_shared<ObservationModel>(
      ObservationModel::create_observation_noise_covariance_matrix(0.02f));  // 0.0009f));//TUNE
  //
  //
  //
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
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
  this->_visualization_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map", 10);
  _adapter_ = adapter_map.at(_adapter_name_)(std::shared_ptr<SENode>(this));
}

/*---------------------- Subscriptions --------------------*/

void SENode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;
  if (this->_perception_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
    return;
  }
  // std::lock_guard lock(this->_mutex_);  // BLOCK IF PREDICTION STEP IS ON GOING
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
  this->_publish_vehicle_state();
  this->_publish_map();
  // sleep(20);
}

void SENode::_imu_subscription_callback(const sensor_msgs::msg::Imu &imu_msg) {
  if (this->_use_odometry_) {
    return;
  }
  double ax = imu_msg.linear_acceleration.x;
  // double ay = imu_msg.linear_acceleration.y;

  double v_rot = imu_msg.angular_velocity.z;

  double angle = this->_ekf_->get_state()(2);

  double ax_map = ax * cos(angle) /* - ay * sin(angle)*/;
  double ay_map = ax * sin(angle) /* + ay * cos(angle) */;

  // print the acceleration in both frames

  // RCLCPP_DEBUG(this->get_logger(), "SUB - Raw from IMU: ax:%f  - v_rot:%f", ax, v_rot);
  // RCLCPP_DEBUG(this->get_logger(), "SUB - translated from IMU: ax:%f - ay:%f - v_rot:%f", ax_map,
  //              ay_map, v_rot);

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

  this->_publish_vehicle_state();
  this->_publish_map();
}

void SENode::_wheel_speeds_subscription_callback(double rl_speed, double fl_speed, double rr_speed,
                                                 double fr_speed, double steering_angle,
                                                 const rclcpp::Time &timestamp) {
  // std::lock_guard lock(this->_mutex_);  // BLOCK IF PREDICTION STEP IS ON GOING
  // RCLCPP_DEBUG(this->get_logger(), "PREDICTION STEP");
  // RCLCPP_DEBUG(this->get_logger(),
  //              "SUB - Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - "
  //              "steering: %f",
  //              rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
  auto [linear_velocity, angular_velocity] =
      common_lib::vehicle_dynamics::odometry_to_velocities_transform(rl_speed, fl_speed, rr_speed,
                                                                     fr_speed, steering_angle);
  MotionUpdate motion_prediction_data;
  motion_prediction_data.translational_velocity = linear_velocity;
  motion_prediction_data.rotational_velocity = angular_velocity;
  this->_motion_update_->translational_velocity = motion_prediction_data.translational_velocity;
  this->_motion_update_->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update_->steering_angle = steering_angle;
  this->_motion_update_->last_update = timestamp;
  // RCLCPP_DEBUG(this->get_logger(), "SUB - translated from wheel speeds: v:%f - w:%f",
  //              this->_motion_update_->translational_velocity,
  //              this->_motion_update_->rotational_velocity);

  if (this->_ekf_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);

  this->_ekf_->prediction_step(temp_update, "wheel_speed_sensor");
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);
  this->_publish_vehicle_state();
  this->_publish_map();
}
/*---------------------- Publications --------------------*/

void SENode::_publish_vehicle_state() {
  if (this->_vehicle_state_ == nullptr) {
    // RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::VehicleState();
  message.position.x = this->_vehicle_state_->pose.position.x;
  message.position.y = this->_vehicle_state_->pose.position.y;
  message.theta = this->_vehicle_state_->pose.orientation;
  float velocity_x = this->_vehicle_state_->velocity_x;
  float velocity_y = this->_vehicle_state_->velocity_y;
  message.linear_velocity = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
  message.angular_velocity = this->_vehicle_state_->rotational_velocity;
  message.header.stamp = this->get_clock()->now();

  // RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f); Velocities: (%f, %f)",
  //              message.position.x, message.position.y, message.theta, message.linear_velocity,
  //              message.angular_velocity);
  this->_vehicle_state_publisher_->publish(message);
}

void SENode::_publish_map() {
  auto cone_array_msg = custom_interfaces::msg::ConeArray();
  auto marker_array_msg = visualization_msgs::msg::MarkerArray();
  // RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  // RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (common_lib::structures::Cone const &cone : *this->_track_map_) {
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position.x = cone.position.x;
    cone_message.position.y = cone.position.y;
    cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
    cone_array_msg.cone_array.push_back(cone_message);
    // RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
    //              cone_message.position.y, cone_message.color.c_str());
  }
  // RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
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
  // RCLCPP_DEBUG(this->get_logger(), "EKF - EFK Step");
}

void SENode::_update_and_publish() {  // Currently unused, as correction step
                                      // and prediction step are carried out
                                      // separately
  this->_ekf_step();
  this->_publish_vehicle_state();
  this->_publish_map();
}