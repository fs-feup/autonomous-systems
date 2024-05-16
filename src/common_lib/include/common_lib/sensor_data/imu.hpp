#pragma once

namespace common_lib::sensor_data {

struct ImuData {
  float rotational_velocity;
  float acceleration_x;
  float acceleration_y;

  ImuData() = default;
  ImuData(float rotational_velocity, float acceleration_x, float acceleration_y);
};
}  // namespace common_lib::sensor_data