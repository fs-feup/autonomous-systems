#include "estimators/kinematic_bicycle_ekf.hpp"

KinematicEKF::KinematicEKF(const VEParameters& params) {
  // Make vx more responsive to observations by reducing its process noise
  this->_process_noise_matrix_(0, 0) = 0;
  this->_process_noise_matrix_(1, 1) = params.imu_acceleration_noise_;
  this->_process_noise_matrix_(2, 2) = params.angular_velocity_process_noise_;
  this->_measurement_noise_matrix_(0, 0) = params.wheel_speed_noise_;
  this->_measurement_noise_matrix_(1, 1) = 10 * params.wheel_speed_noise_;
  this->_measurement_noise_matrix_(2, 2) = params.motor_rpm_noise_;
  this->_measurement_noise_matrix_(3, 3) = params.imu_rotational_noise_;
  this->car_parameters_ = params.car_parameters_;
  this->s2v_model = s2v_models_map.at(params._s2v_model_name_)(params.car_parameters_);
  this->process_model =
      vel_process_models_map.at(params._process_model_name_)(params.car_parameters_);
}

void KinematicEKF::imu_callback(const common_lib::sensor_data::ImuData& imu_data) {
  this->imu_data_ = imu_data;
  if (this->steering_angle_received_) {
    this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
                  this->_last_update_, this->imu_data_, this->steering_angle_);
  }
  if (this->wss_data_received_ && this->motor_rpm_received_) {
    this->correct(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                  this->imu_data_);
  }
  this->imu_data_received_ = true;
}

void KinematicEKF::wss_callback(const common_lib::sensor_data::WheelEncoderData& wss_data) {
  this->wss_data_ = wss_data;
  if (this->imu_data_received_ && this->motor_rpm_received_) {
    this->correct(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                  this->imu_data_);
  }
  this->wss_data_received_ = true;
}

void KinematicEKF::motor_rpm_callback(double motor_rpm) {
  this->motor_rpm_ = motor_rpm;
  if (this->wss_data_received_ && this->imu_data_received_) {
    this->correct(this->_state_, this->_covariance_, this->wss_data_, this->motor_rpm_,
                  this->imu_data_);
  }
  this->motor_rpm_received_ = true;
}

void KinematicEKF::steering_callback(double steering_angle) {
  this->steering_angle_ = steering_angle;
  if (this->imu_data_received_) {
    this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
                  this->_last_update_, this->imu_data_, this->steering_angle_);
  }
  this->steering_angle_received_ = true;
}

common_lib::structures::Velocities KinematicEKF::get_velocities() {
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

void KinematicEKF::predict(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                           const Eigen::Matrix3d& process_noise_matrix,
                           const rclcpp::Time last_update,
                           common_lib::sensor_data::ImuData& imu_data, double steering_angle) {
  rclcpp::Time current_time_point =
      rclcpp::Clock().now();  // TODO: change calculation to use message timestamps

  if (this->_last_update_ == rclcpp::Time(0)) {
    this->_last_update_ = current_time_point;
    return;
  }
  double dt = (current_time_point - last_update).seconds();
  Eigen::Vector3d measurements(imu_data.acceleration_x, imu_data.acceleration_y, steering_angle);

  Eigen::Matrix3d jacobian = this->process_model->get_jacobian_velocities(state, measurements, dt);
  if (!this->_has_made_prediction_) {
    covariance = jacobian * covariance * jacobian.transpose() + process_noise_matrix;
    this->_has_made_prediction_ = true;
  }
  state = this->process_model->get_next_velocities(state, measurements, dt);
  this->_last_update_ = current_time_point;
  RCLCPP_INFO(rclcpp::get_logger("VE"), "After Predict: %f, %f, %f; Readings: %f, %f, %f", state(0),
              state(1), state(2), measurements(0), measurements(1), measurements(2));
}

void KinematicEKF::correct(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                           common_lib::sensor_data::WheelEncoderData& wss_data, double motor_rpm,
                           common_lib::sensor_data::ImuData& imu_data) {
  Eigen::VectorXd predicted_observations = this->s2v_model->cg_velocity_to_wheels(state);
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(4);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, motor_rpm, imu_data.rotational_velocity;
  Eigen::VectorXd y = observations - predicted_observations;
  Eigen::MatrixXd jacobian = this->s2v_model->jacobian_cg_velocity_to_wheels(state);
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->_measurement_noise_matrix_).inverse();

  // Apply state-specific weighting to prefer vx updates
  Eigen::Matrix3d state_weighting = Eigen::Matrix3d::Identity();
  state_weighting(0, 0) = 2.0;  // Make vx twice as responsive
  state_weighting(1, 1) = 0.5;  // Make vy half as responsive
  state_weighting(2, 2) = 0.3;  // Make omega even less responsive

  kalman_gain = state_weighting * kalman_gain;

  // DEBUG PRINTS
  RCLCPP_INFO(rclcpp::get_logger("velocity_estimation"),
              "correct_wheels - Predicted observations: %f %f %f %f", predicted_observations(0),
              predicted_observations(1), predicted_observations(2), predicted_observations(3));
  RCLCPP_INFO(rclcpp::get_logger("velocity_estimation"),
              "correct_wheels - Observations: %f %f %f %f", observations(0), observations(1),
              observations(2), observations(3));
  RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Covariance: \n"
                                                                    << covariance);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - y: \n" << y);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Jacobian: \n"
                                                                    << jacobian);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Kalman gain: \n"
                                                                    << kalman_gain);
  state += kalman_gain * y;
  if (this->_has_made_prediction_) {
    covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
    this->_has_made_prediction_ = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("VE"), "After Correct: %f, %f, %f", state(0), state(1), state(2));
}