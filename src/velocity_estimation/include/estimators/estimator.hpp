#pragma once

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "common_lib/structures/velocities.hpp"

class VelocityEstimator {
public:
  virtual void IMUCallback(const common_lib::sensor_data::ImuData& imu_data) = 0;
  virtual void WSSCallback(const common_lib::sensor_data::WheelEncoderData& wss_data) = 0;
  virtual void ResolverCallback(double resolver_data) = 0;
  virtual void SteeringCallback(double steering_data) = 0;
  virtual common_lib::structures::Velocities get_velocities() = 0;
};