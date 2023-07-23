#include "loc_map/lm_node.hpp"

#include "adapter/adapter.hpp"
#include "utils/car.hpp"
#include "utils/formulas.hpp"

/*---------------------- Subscriptions --------------------*/

LMNode::LMNode(ExtendedKalmanFilter* ekf, Map* perception_map, MotionUpdate* imu_update,
               Map* track_map, VehicleState* vehicle_state, bool use_odometry)
    : Node("loc_map"),
      _ekf(ekf),
      _perception_map(perception_map),
      _motion_update(imu_update),
      _track_map(track_map),
      _vehicle_state(vehicle_state),
      _use_odometry(use_odometry) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMNode::_perception_subscription_callback, this, std::placeholders::_1));
  this->_localization_publisher =
      this->create_publisher<custom_interfaces::msg::Pose>("vehicle_localization", 10);
  this->_mapping_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("track_map", 10);

  new Adapter("eufs", this);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void LMNode::_perception_subscription_callback(const custom_interfaces::msg::ConeArray message) {
  auto cone_array = message.cone_array;
  if (this->_perception_map == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Perception map is null");
    return;
  }
  this->_perception_map->map.clear();
  RCLCPP_DEBUG(this->get_logger(), "SUB - cones from perception:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (auto& cone : cone_array) {
    auto position = Position();
    position.x = cone.position.x;
    position.y = cone.position.y;
    auto color = colors::color_map.at(cone.color);

    RCLCPP_DEBUG(this->get_logger(), "(%f, %f)\t%s", position.x, position.y, cone.color.c_str());

    this->_perception_map->map.insert({position, color});
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");

  // if (this->_mission != Mission::static_inspection_A &&
  //     this->_mission != Mission::static_inspection_B &&
  //     this->_mission != Mission::autonomous_demo) {
  //   this->_update_and_publish();  // Update rate is dictated by perception
  // }
  if (this->_ekf == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - EKF object is null");
    return;
  }
  this->_ekf->correction_step(*(this->_perception_map));
  this->_ekf->update(this->_vehicle_state, this->_track_map);
  // this->_vehicle_state->translational_velocity = temp_update.translational_velocity;
  // this->_vehicle_state->steering_angle = temp_update.steering_angle;
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK correction Step");
  this->_publish_localization();
  this->_publish_map();
}

void LMNode::_imu_subscription_callback(double angular_velocity, double acceleration_x,
                                        double acceleration_y) {
  if (this->_use_odometry) {
    return;
  }
  if (this->_motion_update == nullptr) {
    RCLCPP_WARN(this->get_logger(), "SUB - Motion update object is null");
    return;
  }
  this->_motion_update->rotational_velocity =
      angular_velocity * (180 / M_PI);  // Angular velocity in radians
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_motion_update->last_update)
          .count();
  this->_motion_update->last_update = now;
  this->_motion_update->translational_velocity_y += (acceleration_y * delta) / 1000000;
  this->_motion_update->translational_velocity_x += (acceleration_x * delta) / 1000000;
  this->_motion_update->translational_velocity =
      sqrt(this->_motion_update->translational_velocity_x *
               this->_motion_update->translational_velocity_x +
           this->_motion_update->translational_velocity_y *
               this->_motion_update->translational_velocity_y);
  RCLCPP_DEBUG(this->get_logger(), "SUB - raw from IMU: ax:%f - ay:%f - w:%f", acceleration_x,
               acceleration_y, this->_motion_update->rotational_velocity);
  RCLCPP_DEBUG(this->get_logger(), "SUB - translated from IMU: v:%f - w:%f - vx:%f - vy:%f",
               this->_motion_update->translational_velocity,
               this->_motion_update->rotational_velocity,
               this->_motion_update->translational_velocity_x,
               this->_motion_update->translational_velocity_y);

  if (this->_mission == Mission::static_inspection_A ||
      this->_mission == Mission::static_inspection_B ||
      this->_mission == Mission::autonomous_demo) {
    this->_update_and_publish();
  }
}

/**
 * @brief Transforms the odometry data to velocities
 *
 * @param lb_speed
 * @param lf_speed
 * @param rb_speed
 * @param rf_speed
 * @param steering_angle
 * @return MotionUpdate
 */
MotionUpdate LMNode::odometry_to_velocities_transform(double lb_speed,
                                                      [[maybe_unused]] double lf_speed,
                                                      double rb_speed,
                                                      [[maybe_unused]] double rf_speed,
                                                      double steering_angle) {
  MotionUpdate motion_prediction_data_transformed;
  if (steering_angle == 0) {  // If no steering angle, moving straight
    double lb_velocity = get_wheel_velocity_from_rpm(lb_speed, WHEEL_DIAMETER);
    double rb_velocity = get_wheel_velocity_from_rpm(rb_speed, WHEEL_DIAMETER);
    // double lf_velocity = get_wheel_velocity_from_rpm(lf_speed, WHEEL_DIAMETER); // Simulator
    // double rf_velocity = get_wheel_velocity_from_rpm(rf_speed, WHEEL_DIAMETER); //
    // Are always 0
    motion_prediction_data_transformed.translational_velocity = (lb_velocity + rb_velocity) / 2;
  } else if (steering_angle > 0) {
    double lb_velocity = get_wheel_velocity_from_rpm(lb_speed, WHEEL_DIAMETER);
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    motion_prediction_data_transformed.rotational_velocity =
        lb_velocity / (rear_axis_center_rotation_radius - (AXIS_LENGTH / 2));
    motion_prediction_data_transformed.translational_velocity =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        abs(motion_prediction_data_transformed.rotational_velocity);
  } else {
    double rb_velocity = get_wheel_velocity_from_rpm(rb_speed, WHEEL_DIAMETER);
    double rear_axis_center_rotation_radius = WHEELBASE / tan(steering_angle);
    motion_prediction_data_transformed.rotational_velocity =
        rb_velocity / (rear_axis_center_rotation_radius + (AXIS_LENGTH / 2));
    motion_prediction_data_transformed.translational_velocity =
        sqrt(pow(rear_axis_center_rotation_radius, 2) + pow(REAR_AXIS_TO_CAMERA, 2)) *
        abs(motion_prediction_data_transformed.rotational_velocity);
  }
  return motion_prediction_data_transformed;
}

void LMNode::_wheel_speeds_subscription_callback(double lb_speed, double lf_speed, double rb_speed,
                                                 double rf_speed, double steering_angle) {
  if (!this->_use_odometry) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(),
               "SUB - Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - steering: %f",
               lb_speed, rb_speed, lf_speed, rf_speed, steering_angle);
  MotionUpdate motion_prediction_data = LMNode::odometry_to_velocities_transform(
      lb_speed, lf_speed, rb_speed, rf_speed, steering_angle);
  this->_motion_update->translational_velocity = motion_prediction_data.translational_velocity;
  this->_motion_update->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update->steering_angle = steering_angle;
  this->_motion_update->last_update = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "SUB - translated from wheel speeds: v:%f - w:%f",
               this->_motion_update->translational_velocity,
               this->_motion_update->rotational_velocity);

  // if (this->_mission == Mission::static_inspection_A ||
  //     this->_mission == Mission::static_inspection_B ||
  //     this->_mission == Mission::autonomous_demo) {
  //   this->_update_and_publish();
  // }
  if (this->_ekf == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update);
  this->_ekf->prediction_step(temp_update);
  this->_ekf->update(this->_vehicle_state, this->_track_map);
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK prediction Step");
  this->_publish_localization();
  this->_publish_map();
}

void LMNode::set_mission(Mission mission) {
  if (this->_mission == mission) {
    return;
  }
  this->_mission = mission;
  // Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  // Q(0, 0) = 0.3;
  // Q(1, 1) = 0.3;
  // Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  // R(0, 0) = 0.1;
  // R(1, 1) = 0.1;
  // R(2, 2) = 0.1;
  // MotionModel *motion_model = new NormalVelocityModel(R);
  // ObservationModel observation_model = ObservationModel(Q);
  // this->_ekf = new ExtendedKalmanFilter(*motion_model, observation_model, mission); // Track drive does not need this
}

Mission LMNode::get_mission() { return this->_mission; }

/*---------------------- Publications --------------------*/

void LMNode::_update_and_publish() {
  this->_ekf_step();
  this->_publish_localization();
  this->_publish_map();
}

void LMNode::_publish_localization() {
  if (this->_vehicle_state == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - Vehicle state object is null");
    return;
  }
  auto message = custom_interfaces::msg::Pose();
  const Pose vehicle_localization = (*(this->_vehicle_state)).pose;
  message.position.x = vehicle_localization.position.x;
  message.position.y = vehicle_localization.position.y;
  message.theta = vehicle_localization.orientation;

  RCLCPP_DEBUG(this->get_logger(), "PUB - Pose: (%f, %f, %f)", message.position.x,
               message.position.y, message.theta);
  this->_localization_publisher->publish(message);
}

void LMNode::_publish_map() {
  auto message = custom_interfaces::msg::ConeArray();
  RCLCPP_DEBUG(this->get_logger(), "PUB - cone map:");
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");
  for (auto const& element : this->_track_map->map) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = element.first.x;
    position.y = element.first.y;

    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = colors::color_names[element.second];
    message.cone_array.push_back(cone_message);
    RCLCPP_DEBUG(this->get_logger(), "(%f\t%f)\t%s", cone_message.position.x,
                 cone_message.position.y, cone_message.color.c_str());
  }
  RCLCPP_DEBUG(this->get_logger(), "--------------------------------------");

  this->_mapping_publisher->publish(message);
}

void LMNode::_ekf_step() {
  if (this->_ekf == nullptr) {
    RCLCPP_WARN(this->get_logger(), "PUB - EKF object is null");
    return;
  }
  MotionUpdate temp_update = *(this->_motion_update);
  this->_ekf->prediction_step(temp_update);
  // std::cout << this->_ekf->get_state() << std::endl;
  // std::cout << this->_ekf->get_covariance() << std::endl;
  this->_ekf->correction_step(*(this->_perception_map));
  // std::cout << this->_ekf->get_state() << std::endl;  
  // std::cout << this->_ekf->get_covariance() << std::endl;
  this->_ekf->update(this->_vehicle_state, this->_track_map);
  // this->_vehicle_state->translational_velocity = temp_update.translational_velocity;
  // this->_vehicle_state->steering_angle = temp_update.steering_angle;
  RCLCPP_DEBUG(this->get_logger(), "EKF - EFK Step");
}