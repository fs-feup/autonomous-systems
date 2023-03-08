#include "loc_map/lm_publisher.hpp"

LMPublisher::LMPublisher(Localization* vehicle_localization, Map* map)
    : Node("loc_map_publisher"),
      _vehicle_localization(vehicle_localization),
      _track_map(map),
      _count(0) {
  this->_localization_publisher =
      this->create_publisher<custom_interfaces::msg::Pose>("vehicle_localization", 10);
  this->_mapping_publisher =
      this->create_publisher<custom_interfaces::msg::ConeArray>("track_map", 10);
  this->_timer = this->create_wall_timer(500ms, std::bind(&LMPublisher::_timer_callback, this));
}

void LMPublisher::_timer_callback() {
  this->_publish_localization(*(this->_vehicle_localization));
  this->_publish_map(*(this->_track_map));
}

void LMPublisher::_publish_localization(Localization vehicle_localization) {
  auto message = custom_interfaces::msg::Pose();
  message.position.x = vehicle_localization.position.x;
  message.position.y = vehicle_localization.position.y;
  message.theta = vehicle_localization.orientation;

  RCLCPP_INFO(this->get_logger(), "X:%f - Y:%f - Theta:%f", message.position.x, message.position.y,
              message.theta);
  this->_localization_publisher->publish(message);
}

void LMPublisher::_publish_map(Map track_map) {
  auto message = custom_interfaces::msg::ConeArray();
  for (auto const& element : track_map.map) {
    auto position = custom_interfaces::msg::Point2d();
    position.x = element.first.x;
    position.y = element.first.y;
    auto cone_message = custom_interfaces::msg::Cone();
    cone_message.position = position;
    cone_message.color = colors::color_names[element.second];
    message.cone_array.push_back(cone_message);
  }
  for (auto& cone : message.cone_array) {
    RCLCPP_INFO(this->get_logger(), "Color:%s - Position.x:%f - Position.y:%f", cone.color.c_str(),
                cone.position.x, cone.position.y);
  }
  this->_mapping_publisher->publish(message);
}