#include "estimators/no_rear_wss_ekf.hpp"

NoRearWSSEKF::NoRearWSSEKF(const VEParameters& params) {
  this->_process_noise_matrix_ = Eigen::Matrix3d::Zero();
  this->_process_noise_matrix_(0, 0) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(1, 1) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(2, 2) = params.angular_velocity_process_noise_;
  this->_wheels_measurement_noise_matrix_ = Eigen::MatrixXd::Identity(4, 4);
  this->_wheels_measurement_noise_matrix_(0, 0) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(1, 1) = params.wheel_speed_noise_;
  this->_wheels_measurement_noise_matrix_(2, 2) = params.steering_angle_noise_;
  this->_wheels_measurement_noise_matrix_(3, 3) = params.motor_rpm_noise_;
  this->_imu_measurement_noise_matrix_ = Eigen::MatrixXd(1, 1);
  this->_imu_measurement_noise_matrix_(0, 0) = params.imu_rotational_noise_;
  this->car_parameters_ = params.car_parameters_;
  this->observation_model_ =
      ve_observation_models_map.at(params._ve_observation_model_name_)(params.car_parameters_);
  this->process_model = vel_process_models_map.at(params._process_model_name_)();
}

void NoRearWSSEKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "1 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "1 - Covariance: \n"
                                                                     << this->_covariance_);
  if (!this->imu_data_received_) {
    this->imu_data_received_ = true;
  } else {
    this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
                  this->_last_update_, this->imu_data_);
  }
  this->_last_update_ = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "2 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "2 - Covariance: \n"
                                                                     << this->_covariance_);

  this->correct_imu(this->_state_, this->_covariance_, this->imu_data_);
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                     << this->_covariance_);
}

void NoRearWSSEKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  this->wss_data_received_ = true;
  if (this->steering_angle_received_ && this->motor_rpm_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

void NoRearWSSEKF::motor_rpm_callback(double motor_rpm) {
  this->motor_rpm_ = motor_rpm;
  this->motor_rpm_received_ = true;
  if (this->steering_angle_received_ && this->wss_data_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

void NoRearWSSEKF::steering_callback(double steering_angle) {
  this->steering_angle_ = steering_angle;
  this->steering_angle_received_ = true;
  if (this->wss_data_received_ && this->motor_rpm_received_) {
    this->correct_wheels(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                         this->steering_angle_);
    this->wss_data_received_ = false;
    this->steering_angle_received_ = false;
    this->motor_rpm_received_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "3 - State: %f %f %f", this->_state_(0),
                 this->_state_(1), this->_state_(2));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "3 - Covariance: \n"
                                                                       << this->_covariance_);
  }
}

common_lib::structures::Velocities NoRearWSSEKF::get_velocities() {
  common_lib::structures::Velocities velocities;
  velocities.velocity_x = this->_state_(0);
  velocities.velocity_y = this->_state_(1);
  velocities.rotational_velocity = this->_state_(2);
  velocities.timestamp_ = rclcpp::Clock().now();
  velocities.velocity_x_noise_ = this->_covariance_(0, 0);
  velocities.velocity_y_noise_ = this->_covariance_(1, 1);
  velocities.rotational_velocity_noise_ = this->_covariance_(2, 2);
  return velocities;
}

void NoRearWSSEKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                           const Eigen::Matrix3d& process_noise_matrix,
                           const rclcpp::Time last_update,
                           common_lib::sensor_data::ImuData& imu_data) {
  rclcpp::Time current_time_point =
      rclcpp::Clock().now();  // TODO: change calculation to use message timestamps
  double dt = (current_time_point - last_update).seconds();
  Eigen::Vector3d accelerations(imu_data.acceleration_x, imu_data.acceleration_y, 0.0);

  // Process noise for angular velocity greater, the greater the angular velocity
  Eigen::Matrix3d actual_process_noise_matrix = process_noise_matrix;

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict - Process noise matrix: \n"
                          << actual_process_noise_matrix);

  Eigen::Matrix3d jacobian = this->process_model->get_jacobian_velocities(state, accelerations, dt);
  covariance = jacobian * covariance * jacobian.transpose() + actual_process_noise_matrix;
  state = this->process_model->get_next_velocities(state, accelerations, dt);
  this->_has_made_prediction_ = true;
}

void NoRearWSSEKF::correct_wheels(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                                  common_lib::sensor_data::WheelEncoderData& wss_data,
                                  double motor_rpm, double steering_angle) {
  Eigen::VectorXd predicted_observations = this->observation_model_->expected_observations(state);
  Eigen::VectorXd used_predicted_observations(4);
  used_predicted_observations << predicted_observations(0), predicted_observations(1),
      predicted_observations(4), predicted_observations(5);  // Skip rear wss
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(4);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, steering_angle, motor_rpm;
  Eigen::VectorXd y = observations - used_predicted_observations;
  Eigen::MatrixXd jacobian = this->observation_model_->expected_observations_jacobian(state);
  Eigen::MatrixXd used_jacobian = Eigen::MatrixXd::Zero(4, 3);
  used_jacobian.block(0, 0, 2, 3) = jacobian.block(0, 0, 2, 3);  // Skip rear wss
  used_jacobian.block(2, 0, 2, 3) = jacobian.block(4, 0, 2, 3);  // Skip rear wss
  Eigen::MatrixXd kalman_gain = covariance * used_jacobian.transpose() *
                                (used_jacobian * covariance * used_jacobian.transpose() +
                                 this->_wheels_measurement_noise_matrix_)
                                    .inverse();

  // DEBUG PRINTS
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Predicted observations: %f %f %f %f",
               used_predicted_observations(0), used_predicted_observations(1),
               used_predicted_observations(2), used_predicted_observations(3));
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Observations: %f %f %f %f %f %f", observations(0), observations(1),
               observations(2), observations(3), observations(4), observations(5));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Jacobian: \n"
                                                                     << used_jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Kalman gain: \n"
                                                                     << kalman_gain);
  state += kalman_gain * y;
  if (this->_has_made_prediction_)
    covariance = (Eigen::Matrix3d::Identity() - kalman_gain * used_jacobian) * covariance;
}

void NoRearWSSEKF::correct_imu(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                               common_lib::sensor_data::ImuData& imu_data) {
  Eigen::VectorXd y = Eigen::VectorXd(1);
  y(0) = imu_data.rotational_velocity - state(2);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd(1, 3);
  jacobian(0, 0) = 0;
  jacobian(0, 1) = 0;
  jacobian(0, 2) = 1;
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->_imu_measurement_noise_matrix_)
          .inverse();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Jacobian: \n"
                                                                     << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_imu - Kalman gain: \n"
                                                                     << kalman_gain);
  state += kalman_gain * y;
  if (this->_has_made_prediction_)
    covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
}