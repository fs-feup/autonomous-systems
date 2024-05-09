#include "ros_node/se_node.hpp"

#include "adapter_ekf_state_est/eufs.hpp"
#include "adapter_ekf_state_est/fsds.hpp"
#include "adapter_ekf_state_est/map.hpp"
#include "common_lib/maths/transformations.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/position.hpp"
#include "common_lib/vehicle_dynamics/bicycle_model.hpp"
#include "common_lib/vehicle_dynamics/car_parameters.hpp"

/*---------------------- Constructor --------------------*/

SENode::SENode() : Node("ekf_state_est") {
  this->_use_odometry_ = this->declare_parameter("use_odometry", false);
  std::string adapter_name = this->declare_parameter("adapter", "fsds");
  std::string motion_model_name = this->declare_parameter("motion_model", "normal_velocity");
  std::shared_ptr<MotionModel> motion_model = motion_model_constructors.at(motion_model_name)(
      motion_model_noise_matrixes.at(motion_model_name));
  std::shared_ptr<ObservationModel> observation_model =
      std::make_shared<ObservationModel>(observation_model_noise_matrixes.at("default"));
  std::shared_ptr<DataAssociationModel> data_association_model =
      data_association_model_constructors.at("simple_ml")(71.0);
  _ekf_ = std::make_shared<ExtendedKalmanFilter>(*motion_model, *observation_model,
                                                 *data_association_model);

  _perception_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _motion_update_ = std::make_shared<MotionUpdate>();
  _track_map_ = std::make_shared<std::vector<common_lib::structures::Cone>>();
  _vehicle_state_ = std::make_shared<common_lib::structures::VehicleState>();
  _motion_update_->last_update = this->get_clock()->now();

  this->_perception_subscription_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "/perception/cones", 10,
      std::bind(&SENode::_perception_subscription_callback, this, std::placeholders::_1));
  this->_localization_publisher_ =
      this->create_publisher<custom_interfaces::msg::Pose>("/state_estimation/vehicle_state", 10);
  this->_map_publisher_ =
      this->create_publisher<custom_interfaces::msg::ConeArray>("/state_estimation/map", 10);

  _adapter_ = adapter_map.at(adapter_name)(std::shared_ptr<SENode>(this));
}

/*---------------------- Subscriptions --------------------*/

void SENode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray &msg) {
  auto const &cone_array = msg.cone_array;
  if (this->_perception_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
    return;
  }
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
  this->_publish_localization();
  this->_publish_map();
}

// Currently not utilized
void SENode::_imu_subscription_callback(double angular_velocity, double acceleration_x,
                                        double acceleration_y, rclcpp::Time timestamp) {
  RCLCPP_WARN(this->get_logger(), "TODO: Implement IMU subscription callback properly");
  // if (this->_use_odometry_) {
  //   return;
  // }
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

void SENode::_wheel_speeds_subscription_callback(double lb_speed, double lf_speed, double rb_speed,
                                                 double rf_speed, double steering_angle,
                                                 rclcpp::Time timestamp) {
  if (!this->_use_odometry_) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(),
               "SUB - Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - "
               "steering: %f",
               lb_speed, rb_speed, lf_speed, rf_speed, steering_angle);
  std::pair<double, double> velocities =
      common_lib::vehicle_dynamics::odometry_to_velocities_transform(lb_speed, lf_speed, rb_speed,
                                                                     rf_speed, steering_angle);
  MotionUpdate motion_prediction_data;
  motion_prediction_data.translational_velocity = velocities.first;
  motion_prediction_data.rotational_velocity = velocities.second;
  this->_motion_update_->translational_velocity = motion_prediction_data.translational_velocity;
  this->_motion_update_->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update_->steering_angle = steering_angle;
  this->_motion_update_->last_update = timestamp;
  RCLCPP_DEBUG(this->get_logger(), "SUB - translated from wheel speeds: v:%f - w:%f",
               this->_motion_update_->translational_velocity,
               this->_motion_update_->rotational_velocity);

  if (this->_ekf_ == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "PUB - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update_);
  this->_ekf_->prediction_step(temp_update);
  this->_ekf_->update(this->_vehicle_state_, this->_track_map_);
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK prediction Step");
  this->_publish_localization();
  this->_publish_map();
}

void SENode::set_mission(common_lib::competition_logic::Mission mission) {
  if (this->_mission_ == mission) {
    return;
  }
  this->_mission_ = mission;
  return;
}

/*---------------------- Publications --------------------*/

void SENode::_update_and_publish() {  // Currently unused, as correction step
                                      // and prediction step are carried out
                                      // separately
  this->_ekf_step();
  this->_publish_localization();
  this->_publish_map();
}

void SENode::_publish_localization() {
  if (this->_vehicle_state_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::Pose();
  const common_lib::structures::Pose vehicle_localization = (*(this->_vehicle_state_)).pose;
  message.position.x = vehicle_localization.position.x;
  message.position.y = vehicle_localization.position.y;
  message.theta = vehicle_localization.orientation;

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f)", message.position.x,
               message.position.y, message.theta);
  this->_localization_publisher_->publish(message);
}

void SENode::_publish_map() {
  auto message = custom_interfaces::msg::ConeArray();
  RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (common_lib::structures::Cone const &cone : *this->_track_map_) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = cone.position.x;
    position.y = cone.position.y;

    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = common_lib::competition_logic::get_color_string(cone.color);
    message.cone_array.push_back(cone_message);
    RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
                 cone_message.position.y, cone_message.color.c_str());
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");

  this->_map_publisher_->publish(message);
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
