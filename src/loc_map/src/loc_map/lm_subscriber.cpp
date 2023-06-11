#include "loc_map/lm_subscriber.hpp"

LMSubscriber::LMSubscriber(Map* map, ImuUpdate* imu_update)
    : Node("loc_map_subscriber"), _map(map), _imu_update(imu_update) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_perception_subscription_callback, this, std::placeholders::_1));
  this->_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
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
  this->_imu_update->rotational_velocity = message.angular_velocity.z;
  float acceleration_x = message.linear_acceleration.x;
  float acceleration_y = message.linear_acceleration.y;
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  float delta =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - this->_imu_update->last_update)
          .count();
  this->_imu_update->last_update = now;
  this->_imu_update->delta_time = delta;
  float translational_velocity_x = acceleration_x * delta;
  float translational_velocity_y = acceleration_y * delta;
  this->_imu_update->translational_velocity =
      sqrt(translational_velocity_x * translational_velocity_x +
           translational_velocity_y * translational_velocity_y);
  RCLCPP_INFO(this->get_logger(), "New estimated state from IMU: v:%f - w:%f",
              this->_imu_update->translational_velocity, this->_imu_update->rotational_velocity);
}