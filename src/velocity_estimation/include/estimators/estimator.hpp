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
  /**
   * @brief callback for IMU data that the VENode should call when new IMU data is received
   */
  virtual void imu_callback(const common_lib::sensor_data::ImuData& imu_data) = 0;
  /**
   * @brief callback for wheel speed sensor data that the VENode should call when new wheel speed
   * sensor data is received
   */
  virtual void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) = 0;
  /**
   * @brief callback for motor RPM data that the VENode should call when new motor RPM data is
   * received
   */
  virtual void motor_rpm_callback(double motor_rpm) = 0;
  /**
   * @brief callback for steering angle data that the VENode should call when new steering angle
   * data is received
   */
  virtual void steering_callback(double steering_angle) = 0;
  virtual common_lib::structures::Velocities get_velocities() = 0;
};