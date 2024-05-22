#include "ros_node/se_node.hpp"

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
  std::string adapter_name = this->declare_parameter("adapter", "fsds");
  std::string motion_model_name = this->declare_parameter("motion_model", "normal_velocity_model");
  float observation_noise = static_cast<float>(this->declare_parameter("observation_noise", 0.01f));
  float wheel_speed_sensor_noise =
      static_cast<float>(this->declare_parameter("wheel_speed_sensor_noise", 0.1f));
  float data_association_limit_distance =
      static_cast<float>(this->declare_parameter("data_association_limit_distance", 71.0f));

  std::shared_ptr<MotionModel> motion_model = motion_model_constructors.at(motion_model_name)(
      MotionModel::create_process_noise_covariance_matrix(wheel_speed_sensor_noise));
  std::shared_ptr<ObservationModel> observation_model = std::make_shared<ObservationModel>(
      ObservationModel::create_observation_noise_covariance_matrix(observation_noise));
  std::shared_ptr<DataAssociationModel> data_association_model =
      data_association_model_constructors.at("simple_ml")(data_association_limit_distance);
  _ekf_ = std::make_shared<ExtendedKalmanFilter>(motion_model, observation_model,
                                                 data_association_model);

  _perception_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _motion_update_ = std::make_shared<MotionUpdate>();
  _track_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _vehicle_state_ = std::make_shared<common_lib::structures::VehicleState>();
  _motion_update_->last_update = this->get_clock()->now();

  if (!_use_simulated_perception_) {
    this->_perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "/perception/cones", 10,
        std::bind(&SENode::_perception_subscription_callback, this, std::placeholders::_1));
  }
  this->_vehicle_state_publisher_ = this->create_publisher<custom_interfaces::msg::VehicleState>(
      "/state_estimation/vehicle_state", 10);
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);
  this->_visualization_map_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/state_estimation/visualization_map", 10);
  _adapter_ = adapter_map.at(adapter_name)(std::shared_ptr<SENode>(this));
}

/*---------------------- Subscriptions --------------------*/

void SENode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;
  if (this->_perception_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
    return;
  }
  std::lock_guard lock(this->_mutex_);  // BLOCK IF PREDICTION STEP IS ON GOING
  RCLCPP_DEBUG(this->get_logger(), "CORRECTION STEP");
  this->_perception_map_->clear();
  RCLCPP_DEBUG(this->get_logger(), "SUB - cones from perception:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");

  for (auto &cone : cone_array) {
    RCLCPP_DEBUG(this->get_logger(), "(%f, %f)\t%s", cone.position.x, cone.position.y,
                 cone.color.c_str());

    this->_perception_map_->push_back(common_lib::structures::Cone(cone.position.x, cone.position.y,
                                                                   cone.color, cone.confidence));
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");

  if (this->_ekf_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  this->_ekf_->correction_step(*(this->_perception_map_));
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK correction Step");
  this->_publish_vehicle_state();
  this->_publish_map();
  RCLCPP_DEBUG(this->get_logger(), "CORRECTION STEP END\n\n");
}

// Currently not utilized
void SENode::_imu_subscription_callback(const sensor_msgs::msg::Imu &imu_msg) {
  if (this->_use_odometry_) {
    return;
  }
  RCLCPP_WARN(this->get_logger(), "TODO: Implement IMU subscription callback properly");
  // if (this->_motion_update_ == nullptr) {
  //   RCLCPP_WARN(this->get_logger(), "ATTR - Motion update object is null");
  //   return;
  // }
  // this->_motion_update_->rotational_velocity =
  //     angular_velocity * (180 / M_PI);  // Angular velocity in radians
  // std::chrono::time_point<std::chrono::high_resolution_clock> now =
  //     std::chrono::high_resolution_clock::now();
  // double delta = std::chrono::duration_cast<std::chrono::microseconds>(
  //                    now - this->_motion_update_->last_update)
  //                    .count();
  // this->_motion_update_->last_update = now;  // WRONG
  // this->_motion_update_->translational_velocity_y += (acceleration_y * delta) / 1000000;
  // this->_motion_update_->translational_velocity_x += (acceleration_x * delta) / 1000000;
  // this->_motion_update_->translational_velocity =
  //     sqrt(this->_motion_update_->translational_velocity_x *
  //              this->_motion_update_->translational_velocity_x +
  //          this->_motion_update_->translational_velocity_y *
  //              this->_motion_update_->translational_velocity_y);
  // RCLCPP_DEBUG(this->get_logger(), "SUB - raw from IMU: ax:%f - ay:%f - w:%f", acceleration_x,
  //              acceleration_y, this->_motion_update_->rotational_velocity);
  // RCLCPP_DEBUG(this->get_logger(), "SUB - translated from IMU: v:%f - w:%f - vx:%f - vy:%f",
  //              this->_motion_update_->translational_velocity,
  //              this->_motion_update_->rotational_velocity,
  //              this->_motion_update_->translational_velocity_x,
  //              this->_motion_update_->translational_velocity_y);
}

void SENode::_wheel_speeds_subscription_callback(double rl_speed, double fl_speed, double rr_speed,
                                                 double fr_speed, double steering_angle,
                                                 const rclcpp::Time &timestamp) {
  if (!this->_use_odometry_) {
    return;
  }
  std::lock_guard lock(this->_mutex_);  // BLOCK IF PREDICTION STEP IS ON GOING
  RCLCPP_DEBUG(this->get_logger(), "PREDICTION STEP");
  RCLCPP_DEBUG(this->get_logger(),
               "SUB - Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - "
               "steering: %f",
               rl_speed, rr_speed, fl_speed, fr_speed, steering_angle);
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
  RCLCPP_DEBUG(this->get_logger(), "SUB - translated from wheel speeds: v:%f - w:%f",
               this->_motion_update_->translational_velocity,
               this->_motion_update_->rotational_velocity);

  if (this->_ekf_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "ATTR - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);
  this->_ekf_->prediction_step(temp_update);
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK prediction Step");
  this->_publish_vehicle_state();
  this->_publish_map();
  RCLCPP_DEBUG(this->get_logger(), "PREDICTION STEP END\n\n");
}
/*---------------------- Publications --------------------*/

void SENode::_publish_vehicle_state() {
  if (this->_vehicle_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::VehicleState();
  message.position.x = this->_vehicle_state_->pose.position.x;
  message.position.y = this->_vehicle_state_->pose.position.y;
  message.theta = this->_vehicle_state_->pose.orientation;
  message.linear_velocity = this->_vehicle_state_->linear_velocity;
  message.angular_velocity = this->_vehicle_state_->angular_velocity;
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
      *this->_track_map_, "map_cones", "map");
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
  this->_ekf_->prediction_step(temp_update);
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