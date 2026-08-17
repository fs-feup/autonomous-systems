#pragma once

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

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
  std::shared_ptr<SEParameters> params_;

  Eigen::Matrix<double, StateSize, StateSize> process_noise_matrix_;
  Eigen::MatrixXd measurement_noise_matrix_;

  std::shared_ptr<ProcessModel> process_model_;
  // One process-model instance per prediction thread; the models cache into members, so each
  // thread needs its own. pool_[0] aliases process_model_.
  std::vector<std::shared_ptr<ProcessModel>> process_model_pool_;
  int prediction_threads_ = 1;
  VehicleState process_model_data_;
  std::shared_ptr<ObservationModel> observation_model_;

  rclcpp::Time last_update_;
  bool received_any_measurement_ = false;  // startup gate: hold until a first row survives
  Eigen::Vector4d execution_times_;

  Eigen::VectorXd weights_m_;
  double lambda_;

  // Precomputed constant: (1 + beta - alpha^2), used in every covariance update
  double w0_cov_correction_;

  // Cached measurement size (fixed at construction)
  int meas_size_;

  // Preallocated workspace matrices
  Eigen::Matrix<double, 2 * StateSize + 1, StateSize, Eigen::RowMajor> sigma_points_;
  Eigen::Matrix<double, 2 * StateSize + 1, StateSize> centered_;
  Eigen::Matrix<double, 2 * StateSize + 1, Eigen::Dynamic, Eigen::RowMajor> predicted_measurements_;
  Eigen::Matrix<double, 2 * StateSize + 1, Eigen::Dynamic, Eigen::RowMajor> centered_measurements_;
  Eigen::MatrixXd cross_covariance_;
  Eigen::MatrixXd kalman_gain_;
  Eigen::MatrixXd predicted_measurement_covariance_;
  Eigen::MatrixXd H_;
  Eigen::VectorXd predicted_measurement_mean_;
  Eigen::MatrixXd G_;

  common_lib::structures::ControlCommand last_control_command_ =
      common_lib::structures::ControlCommand();

  // Thread synchronization mutexes
  std::mutex control_command_mutex_;
  mutable std::mutex observation_model_mutex_;

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
  UKF(std::shared_ptr<SEParameters> se_parameters, std::shared_ptr<ProcessModel> process_model,
      std::shared_ptr<ObservationModel> observation_model);

  void control_callback(const common_lib::structures::ControlCommand& control_command) override;
  void imu_callback(const common_lib::sensor_data::ImuData& imu_data) override;
  void wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void motor_rpm_callback(double motor_rpm, const rclcpp::Time& stamp) override;
  void steering_callback(double steering_angle, const rclcpp::Time& stamp) override;
  void timer_callback(State& curr_state) override;

  VehicleState get_process_model_data() const override { return process_model_data_; }
  Eigen::Vector4d get_exec_times() const override { return execution_times_; }

  std::vector<SensorHealth> get_sensor_health() const override {
    std::lock_guard<std::mutex> lock(observation_model_mutex_);
    return observation_model_->get_health();
  }
};