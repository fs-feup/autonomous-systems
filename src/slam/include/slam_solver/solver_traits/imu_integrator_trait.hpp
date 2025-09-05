#pragma once

#include "common_lib/sensor_data/imu.hpp"

class ImuIntegratorTrait {
public:
  /**
   * @brief Integrate IMU data into the SLAM solver
   *
   * @param pose_difference Pose difference in the form of [dx, dy, dtheta]
   */
  virtual void add_imu_data(const common_lib::sensor_data::ImuData &imu_data) = 0;

  virtual ~ImuIntegratorTrait() = default;
};