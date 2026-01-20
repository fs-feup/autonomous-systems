#include "common_lib/sensor_data/imu.hpp"

namespace common_lib::sensor_data {

ImuData::ImuData(double rotational_velocity, double acceleration_x, double acceleration_y,
                 rclcpp::Time timestamp, double rotational_velocity_noise,
                 double acceleration_x_noise, double acceleration_y_noise)
    : rotational_velocity(rotational_velocity),
      acceleration_x(acceleration_x),
      acceleration_y(acceleration_y),
      rotational_velocity_noise(rotational_velocity_noise),
      acceleration_x_noise(acceleration_x_noise),
      acceleration_y_noise(acceleration_y_noise),
      timestamp_(timestamp) {}

}  // namespace common_lib::sensor_data