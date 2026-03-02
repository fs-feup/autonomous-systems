#pragma once

#include "models/observation/observation_model.hpp"

class ObservationModel02 : public ObservationModel {
  common_lib::sensor_data::ImuData last_imu_data_ = common_lib::sensor_data::ImuData();
  common_lib::sensor_data::WheelEncoderData last_wss_data_ =
      common_lib::sensor_data::WheelEncoderData();
  double last_motor_rpm_ = 0.0;
  double last_steering_angle_ = 0.0;

public:
  ObservationModel02(const std::shared_ptr<SEParameters>& parameters)
      : ObservationModel(parameters) {}
  /**
   * @brief Transforms the state of the vehicle into the 02's sensor observation space.
   * @param state The state of the vehicle.
   * @return expected_observations The expected observations in the order of
   * [acceleration_x, acceleration_y, imu_rotational_velocity, fl_wheel_speed, fr_wheel_speed,
   * rl_wheel_speed, rr_wheel_speed, steering_angle, motor_rpm]
   */
  void expected_observations(const State& state,
                             Eigen::Ref<Eigen::VectorXd> expected_observations) override;

  int get_measurement_size() const override { return 9; }

  void update_imu_data(const common_lib::sensor_data::ImuData& imu_data) override {
    last_imu_data_ = imu_data;
  }
  void update_wss_data(const common_lib::sensor_data::WheelEncoderData& wss_data) override {
    last_wss_data_ = wss_data;
  }
  void update_motor_rpm(double motor_rpm) override { last_motor_rpm_ = motor_rpm; }
  void update_steering_angle(double steering_angle) override {
    last_steering_angle_ = steering_angle;
  }

  Eigen::VectorXd get_last_observations() const override;
};