#include "loc_map/lm_subscriber.hpp"

#include "adapter/adapter.hpp"
#include "utils/car.hpp"
#include "utils/formulas.hpp"

LMSubscriber::LMSubscriber(Map* map, MotionUpdate* imu_update, bool use_odometry)
    : Node("loc_map_subscriber"),
      _map(map),
      _motion_update(imu_update),
      _use_odometry(use_odometry) {
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
  if (this->_use_odometry) {
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
  this->_motion_update->translational_velocity_x +=
      (acceleration_x * delta) / 1000000;  // TODO(marhcouto): check referentials in the ADS
  this->_motion_update->translational_velocity =
      sqrt(this->_motion_update->translational_velocity_x *
               this->_motion_update->translational_velocity_x +
           this->_motion_update->translational_velocity_y *
               this->_motion_update->translational_velocity_y);
  RCLCPP_INFO(this->get_logger(), "Raw from IMU: ax:%f - ay:%f - w:%f", acceleration_x,
              acceleration_y, this->_motion_update->rotational_velocity);
  RCLCPP_INFO(this->get_logger(), "Translated from IMU: v:%f - w:%f - vx:%f - vy:%f",
              this->_motion_update->translational_velocity,
              this->_motion_update->rotational_velocity,
              this->_motion_update->translational_velocity_x,
              this->_motion_update->translational_velocity_y);
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
MotionUpdate LMSubscriber::odometry_to_velocities_transform(double lb_speed, double lf_speed,
                                                            double rb_speed, double rf_speed,
                                                            double steering_angle) {
  MotionUpdate motion_prediction_data_transformed;
  if (steering_angle == 0) {  // If no steering angle, moving straight
    double lb_velocity = get_wheel_velocity_from_rpm(lb_speed, WHEEL_DIAMETER);
    double rb_velocity = get_wheel_velocity_from_rpm(rb_speed, WHEEL_DIAMETER);
    double lf_velocity = get_wheel_velocity_from_rpm(lf_speed, WHEEL_DIAMETER);
    double rf_velocity = get_wheel_velocity_from_rpm(rf_speed, WHEEL_DIAMETER);
    motion_prediction_data_transformed.translational_velocity =
        (lb_velocity + rb_velocity + lf_velocity + rf_velocity) / 4;
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

void LMSubscriber::_wheel_speeds_subscription_callback(double lb_speed, double lf_speed,
                                                       double rb_speed, double rf_speed,
                                                       double steering_angle) {
  if (!this->_use_odometry) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "Raw from wheel speeds: lb:%f - rb:%f - lf:%f - rf:%f - steering: %f", lb_speed,
              rb_speed, lf_speed, rf_speed, steering_angle);
  MotionUpdate motion_prediction_data = LMSubscriber::odometry_to_velocities_transform(
      lb_speed, lf_speed, rb_speed, rf_speed, steering_angle);
  this->_motion_update->translational_velocity = motion_prediction_data.translational_velocity;
  this->_motion_update->rotational_velocity = motion_prediction_data.rotational_velocity;
  this->_motion_update->steering_angle = steering_angle;
  this->_motion_update->last_update = std::chrono::high_resolution_clock::now();
}

void LMSubscriber::set_mission(Mission mission) { this->_mission = mission; }

Mission LMSubscriber::get_mission() { return this->_mission; }