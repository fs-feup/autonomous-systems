#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "utils/state_define.hpp"

/**
 * @brief Interface for observation models
 *
 * This class defines the interface for observation models. Observation models are used to
 * transform the state of the vehicle into the measurement space.
 */
class ObservationModel {
public:
  virtual void transform(State& state, common_lib::sensor_data::ImuData imu_data,
                         common_lib::sensor_data::WheelEncoderData wheel_encoder_data,
                         double steering_angle, double motor_rpm) = 0;

  virtual ~ObservationModel() = default;
};
