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
#include "models/process/map.hpp"
#include "models/sensors/sensor_data.hpp"
#include "motion_lib/vel_process_model/map.hpp"
#include "perception_sensor_lib/observation_model/ve/map.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"

class UKF : public StateEstimator {
  State state_ = State::Zero();
  Eigen::Matrix<double, 10, 10> covariance_ = Eigen::Matrix<double, 10, 10>::Identity();
  SEParameters params_;

  Eigen::Matrix<double, 10, 10> process_noise_matrix_;
  Eigen::Matrix<double, 10, 10> wheels_measurement_noise_matrix_;
  Eigen::Matrix<double, 10, 10> imu_measurement_noise_matrix_;
  Eigen::Matrix<double, 10, 10> motor_rpm_measurement_noise_matrix_;
  Eigen::Matrix<double, 10, 10> steering_angle_measurement_noise_matrix_;

  std::shared_ptr<ProcessModel> process_model_;

  const rclcpp::Time last_update_;
  std::priority_queue<std::unique_ptr<SensorData>> sensor_data_queue_;

  Eigen::VectorXd weights_;
  double lambda_;

  void compute_sigma_points(const State& state, const Eigen::Matrix<double, 10, 10>& covariance,
                            Eigen::Matrix<State, -1, 1>& sigma_points);

  void predict(State& state, Eigen::Matrix<double, 10, 10>& covariance,
               common_lib::structures::ControlCommand control_command);

  void correct(State& state, Eigen::Matrix<double, 10, 10>& covariance,
               const Eigen::Matrix<double, 10, 10>& sensor_noise_matrix,
               const SensorData& sensor_data);

  void get_mean(const Eigen::Matrix<State, -1, 1>& sigma_points, State& mean);

  void get_covariance(const Eigen::Matrix<State, -1, 1>& sigma_points, const State& mean,
                      Eigen::Matrix<double, 10, 10>& covariance);

public:
  UKF(SEParameters se_parameters);
};