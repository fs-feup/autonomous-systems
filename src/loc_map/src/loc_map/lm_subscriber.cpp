#include "loc_map/lm_subscriber.hpp"

LMSubscriber::LMSubscriber(Map* map, VehicleState* vehicle_state)
    : Node("loc_map_subscriber"), _map(map), _vehicle_state(vehicle_state) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_perception_subscription_callback, this, std::placeholders::_1));
  this->_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&LMSubscriber::_imu_subscription_callback, this, std::placeholders::_1));
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
  this->_vehicle_state->rotational_velocity_x = message.angular_velocity.x;
  this->_vehicle_state->rotational_velocity_y = message.angular_velocity.y;
  float acceleration_x = message.linear_acceleration.x;
  float acceleration_y = message.linear_acceleration.y;
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  float delta =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - this->_vehicle_state->last_update)
          .count();
  this->_vehicle_state->last_update = now;
  this->_vehicle_state->translational_velocity_x += acceleration_x * delta;
  this->_vehicle_state->translational_velocity_y += acceleration_y * delta;
  RCLCPP_INFO(this->get_logger(), "New estimated state from IMU: vx:%f - vy:%f - wx:%f - wy:%f",
              this->_vehicle_state->translational_velocity_x,
              this->_vehicle_state->translational_velocity_y,
              this->_vehicle_state->rotational_velocity_x,
              this->_vehicle_state->rotational_velocity_y);
}