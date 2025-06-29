#include "common_lib/sensor_data/imu.hpp"

namespace common_lib::sensor_data {

ImuData::ImuData(float rotational_velocity, float acceleration_x, float acceleration_y,
                 rclcpp::Time timestamp)
    : rotational_velocity(rotational_velocity),
      acceleration_x(acceleration_x),
      acceleration_y(acceleration_y),
      timestamp_(timestamp) {}

}  // namespace common_lib::sensor_data