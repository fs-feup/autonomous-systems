#include "loc_map/lm_subscriber.hpp"

LMSubscriber::LMSubscriber(Map* map, ImuUpdate* imu_update, OdometryUpdate* odometry_update)
    : Node("loc_map_subscriber"),
      _map(map),
      _imu_update(imu_update),
      _odometry_update(odometry_update) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_perception_subscription_callback, this, std::placeholders::_1));
  this->_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&LMSubscriber::_imu_subscription_callback, this, std::placeholders::_1));
  this->_wheel_speeds_subscription = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
      "ros_can/wheel_speeds", 10,
      std::bind(&LMSubscriber::_wheel_speeds_subscription_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscriber started");
}

void LMSubscriber::_perception_subscription_callback(
    const custom_interfaces::msg::ConeArray message) {
  auto cone_array = message.cone_array;
  this->_map->map.clear();
  RCLCPP_INFO(this->get_logger(), "Cones from perception:\n--------------------------------------");
  for (auto& cone : cone_array) {
    auto position = Position();
    position.x = cone.position.x;
    position.y = cone.position.y;
    auto color = colors::color_map.at(cone.color);

    RCLCPP_INFO(this->get_logger(), "(%f, %f)\t%s", position.x, position.y, cone.color.c_str());

    this->_map->map.insert({position, color});
  }
  RCLCPP_INFO(this->get_logger(), "--------------------------------------");
}

void LMSubscriber::_imu_subscription_callback(const sensor_msgs::msg::Imu message) {
  this->_imu_update->rotational_velocity =
      message.angular_velocity.z * (180 / M_PI);  // Angular velocity in radians
  double acceleration_x = message.linear_acceleration.x;
  double acceleration_y = message.linear_acceleration.y;
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  double delta =
      std::chrono::duration_cast<std::chrono::microseconds>(now - this->_imu_update->last_update)
          .count();
  this->_imu_update->last_update = now;
  this->_imu_update->translational_velocity_y += (acceleration_y * delta) / 1000000;
  this->_imu_update->translational_velocity_x +=
      (acceleration_x * delta) / 1000000;  // TODO(marhcouto): check referentials in the ADS
  this->_imu_update->translational_velocity = sqrt(
      this->_imu_update->translational_velocity_x * this->_imu_update->translational_velocity_x +
      this->_imu_update->translational_velocity_y * this->_imu_update->translational_velocity_y);
  RCLCPP_INFO(this->get_logger(), "Raw from IMU: ax:%f - ay:%f - w:%f", acceleration_x,
              acceleration_y, this->_imu_update->rotational_velocity);
  RCLCPP_INFO(this->get_logger(), "Translated from IMU: v:%f - w:%f - vx:%f - vy:%f",
              this->_imu_update->translational_velocity, this->_imu_update->rotational_velocity,
              this->_imu_update->translational_velocity_x,
              this->_imu_update->translational_velocity_y);
}

void LMSubscriber::_wheel_speeds_subscription_callback(
    const eufs_msgs::msg::WheelSpeedsStamped message) {
  this->_odometry_update->lb_speed = message.speeds.lb_speed;
  this->_odometry_update->lf_speed = message.speeds.lf_speed;
  this->_odometry_update->rb_speed = message.speeds.rb_speed;
  this->_odometry_update->rf_speed = message.speeds.rf_speed;
  this->_odometry_update->steering_angle = message.speeds.steering;

  RCLCPP_INFO(this->get_logger(),
              "Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - steering: %f",
              this->_odometry_update->lb_speed, this->_odometry_update->rb_speed,
              this->_odometry_update->lf_speed, this->_odometry_update->rf_speed,
              this->_odometry_update->steering_angle);
}