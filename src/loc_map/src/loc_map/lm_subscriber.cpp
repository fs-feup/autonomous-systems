#include "loc_map/lm_subscriber.hpp"

#include "adapter/adapter.hpp"

LMSubscriber::LMSubscriber(Map* map, ImuUpdate* imu_update, OdometryUpdate* odometry_update)
    : Node("loc_map_subscriber"),
      _map(map),
      _imu_update(imu_update),
      _odometry_update(odometry_update) {
  this->_perception_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
      "perception/cone_coordinates", 10,
      std::bind(&LMSubscriber::_perception_subscription_callback, this, std::placeholders::_1));

  new Adapter("eufs", this);

  RCLCPP_INFO(this->get_logger(), "Subscriber started");
}

void LMSubscriber::_perception_subscription_callback(
    const custom_interfaces::msg::ConeArray message) {
  auto cone_array = message.cone_array;
  this->_map->map.clear();
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

void LMSubscriber::_imu_subscription_callback(double angular_velocity, double acceleration_x,
                                              double acceleration_y) {
  this->_imu_update->rotational_velocity =
      angular_velocity * (180 / M_PI);  // Angular velocity in radians
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

void LMSubscriber::_wheel_speeds_subscription_callback(double lb_speed, double lf_speed,
                                                       double rb_speed, double rf_speed,
                                                       double steering_angle) {
  this->_odometry_update->lb_speed = lb_speed;
  this->_odometry_update->lf_speed = lf_speed;
  this->_odometry_update->rb_speed = rb_speed;
  this->_odometry_update->rf_speed = rf_speed;
  this->_odometry_update->steering_angle = steering_angle;

  RCLCPP_INFO(this->get_logger(),
              "Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - steering: %f",
              this->_odometry_update->lb_speed, this->_odometry_update->rb_speed,
              this->_odometry_update->lf_speed, this->_odometry_update->rf_speed,
              this->_odometry_update->steering_angle);
}

void LMSubscriber::set_mission(Mission mission) {
    this->_mission = mission;
}

Mission LMSubscriber::get_mission() {
    return this->_mission;
}