#pragma once

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "common_lib/structures/velocities.hpp"
/**
 * @brief Interface for velocity estimators
 *
 * This class defines the interface for velocity estimators. Velocity estimators are used to
 * estimate the vehicle's velocity based on sensor measurements.
 */
class VelocityEstimator {
public:
  virtual void imu_callback(const common_lib::sensor_data::ImuData& imu_data) = 0;
  virtual void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) = 0;
  virtual void motor_rpm_callback(double motor_rom) = 0;
  virtual void steering_callback(double steering_angle) = 0;
  virtual common_lib::structures::Velocities get_velocities() = 0;
};