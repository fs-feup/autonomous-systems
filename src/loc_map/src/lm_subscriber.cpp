#include "loc_map/lm_subscriber.hpp"

LMSubscriber::LMSubscriber(Map* map) : Node("loc_map_subscriber"), _map(map) {
  this->_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_subscription_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "[LOC_MAP] Subscriber started");
}

void LMSubscriber::_subscription_callback(const custom_interfaces::msg::ConeArray message) {
  auto cone_array = message.cone_array;

  for (auto& cone : cone_array) {
    auto position = Position();
    position.x = cone.position.x;
    position.y = cone.position.y;
    auto color = colors::color_map.at(cone.color);

    RCLCPP_INFO(this->get_logger(), "(%f,\t%f)\t%s", position.x, position.y, cone.color.c_str());

    this->_map->map.insert({position, color});
  }
  RCLCPP_INFO(this->get_logger(), "Map size: %ld", this->_map->map.size());
  RCLCPP_INFO(this->get_logger(), "--------------------------------------");
}