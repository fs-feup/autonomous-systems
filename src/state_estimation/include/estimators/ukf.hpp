#pragma once

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <memory>
#include <queue>
#include <string>

#include "common_lib/structures/control_command.hpp"
#include "common_lib/structures/velocities.hpp"
#include "custom_interfaces/msg/velocities.hpp"
#include "estimators/estimator.hpp"
#include "models/observation/map.hpp"
#include "models/process/map.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"

class UKF : public StateEstimator {
  State state_ = State::Zero();
  Eigen::Matrix<double, StateSize, StateSize> covariance_ =
      Eigen::Matrix<double, StateSize, StateSize>::Identity();
  SEParameters params_;

  Eigen::Matrix<double, StateSize, StateSize> process_noise_matrix_;
  Eigen::MatrixXd measurement_noise_matrix_;

  std::shared_ptr<ProcessModel> process_model_;
  std::shared_ptr<ObservationModel> observation_model_;

  const rclcpp::Time last_update_;

  Eigen::VectorXd weights_;
  double lambda_;

  common_lib::structures::ControlCommand last_control_command_ =
      common_lib::structures::ControlCommand();

  /**
   * @brief Compute the sigma points for the given state and covariance using the Merwe Scaled Sigma
   * Points method.
   * @param state The current state of the system
   * @param covariance The current covariance of the state estimate
   * @param sigma_points The output matrix to store the computed sigma points, should be of size (2
   * * StateSize + 1, StateSize)
   */
  void compute_sigma_points(
      const State& state, const Eigen::Matrix<double, StateSize, StateSize>& covariance,
      Eigen::Matrix<double, 2 * StateSize + 1, StateSize, Eigen::RowMajor>& sigma_points);

  void get_cross_covariance(const Eigen::Matrix<double, 2 * StateSize + 1, StateSize>& sigma_points,
                            const State& mean, const Eigen::VectorXd& expected_observations,
                            Eigen::MatrixXd& cross_covariance);

public:
  UKF(SEParameters se_parameters, std::shared_ptr<ProcessModel> process_model,
      std::shared_ptr<ObservationModel> observation_model);

  void control_callback(const common_lib::structures::ControlCommand& control_command) override;
  void imu_callback(const common_lib::sensor_data::ImuData& imu_data) override;
  void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void motor_rpm_callback(double motor_rpm) override;
  void steering_callback(double steering_angle) override;
  void timer_callback(State& curr_state) override;
};