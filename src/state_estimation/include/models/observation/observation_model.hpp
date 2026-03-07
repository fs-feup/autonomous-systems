#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"
/**
 * @brief Interface for observation models
 *
 * This class defines the interface for observation models. Observation models are used to
 * manage and control observations.
 * Order of the expected observations vector is defined as follows:
 * [acceleration_x, acceleration_y, imu_rotational_velocity, fl_wheel_speed, fr_wheel_speed,
 * rl_wheel_speed, rr_wheel_speed, steering_angle, motor_rpm]
 */
class ObservationModel {
protected:
  std::shared_ptr<SEParameters> parameters_;

public:
  ObservationModel(const std::shared_ptr<SEParameters>& parameters) : parameters_(parameters) {}

  virtual void expected_observations(const State& state,
                                     Eigen::Ref<Eigen::VectorXd> expected_observations) = 0;

  virtual int get_measurement_size() const = 0;

  virtual Eigen::VectorXd get_last_observations() const = 0;

  virtual Eigen::MatrixXd get_last_observations_noise() const = 0;

  virtual void update_imu_data(const common_lib::sensor_data::ImuData& imu_data) = 0;
  virtual void update_wss_data(const common_lib::sensor_data::WheelEncoderData& wss_data) = 0;
  virtual void update_motor_rpm(double motor_rpm) = 0;
  virtual void update_steering_angle(double steering_angle) = 0;

  virtual ~ObservationModel() = default;
};
