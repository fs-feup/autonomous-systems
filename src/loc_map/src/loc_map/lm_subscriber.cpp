#include "loc_map/lm_subscriber.hpp"

LMSubscriber::LMSubscriber(Map* map, ImuUpdate* imu_update)
    : Node("loc_map_subscriber"), _map(map), _imu_update(imu_update) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_perception_subscription_callback, this, std::placeholders::_1));
  this->_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
      std::bind(&LMSubscriber::_imu_subscription_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[LOC_MAP] Subscriber started");
}

void LMSubscriber::_perception_subscription_callback(
    const custom_interfaces::msg::ConeArray message) {
  auto cone_array = message.cone_array;

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
  RCLCPP_INFO(this->get_logger(), "[LOC_MAP] Raw from IMU: ax:%f - ay:%f - w:%f", acceleration_x,
              acceleration_y, this->_imu_update->rotational_velocity);
  RCLCPP_INFO(this->get_logger(), "[LOC_MAP] Translated from IMU: v:%f - w:%f - vx:%f - vy:%f",
              this->_imu_update->translational_velocity, this->_imu_update->rotational_velocity,
              this->_imu_update->translational_velocity_x,
              this->_imu_update->translational_velocity_y);
}