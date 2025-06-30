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
  this->s2v_model = s2v_models_map.at(params._s2v_model_name_)(params.car_parameters_);
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
  }else {
    // Choose prediction method based on steering data availability
    if(this->steering_angle_received_) {
      //this->predict_with_steering(this->_state_, this->_covariance_, 
      //                                this->_process_noise_matrix_, this->_last_update_, 
      //                                this->imu_data_, this->steering_angle_);
    }else {
      //this->predict(this->_state_, this->_covariance_, this->_process_noise_matrix_,
      //                this->_last_update_, this->imu_data_);
    }
  }

  this->_last_update_ = rclcpp::Clock().now();
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"), "2 - State: %f %f %f", this->_state_(0),
               this->_state_(1), this->_state_(2));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "2 - Covariance: \n"
                                                                     << this->_covariance_);

  //this->correct_imu(this->_state_, this->_covariance_, this->imu_data_);
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

void NoRearWSSEKF::predict_with_steering(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                                            const Eigen::Matrix3d& process_noise_matrix,
                                            const rclcpp::Time last_update,
                                            const common_lib::sensor_data::ImuData& imu_data,
                                            double steering_angle) {

  rclcpp::Time current_time_point = rclcpp::Clock().now();
  double dt = (current_time_point - last_update).seconds();
  Eigen::Vector3d accelerations(imu_data.acceleration_x, imu_data.acceleration_y, 0.0);

  // Compute adaptive process noise considering steering angle
  Eigen::Matrix3d actual_process_noise_matrix = 
        this->compute_adaptive_process_noise(process_noise_matrix, steering_angle);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"),
                      "predict_with_steering - Process noise matrix: \n"
                          << actual_process_noise_matrix);

  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "predict_with_steering - Steering angle: %f rad", steering_angle);
 
  // Compute Jacobian for covariance propagation
  Eigen::Matrix3d jacobian = this->compute_steering_jacobian(state, accelerations, dt, steering_angle);

  // Propagate covariance
  covariance = jacobian * covariance * jacobian.transpose() + actual_process_noise_matrix;

  // Update state using steering-aware model
  state = this->compute_steering_state_update(state, accelerations, dt, steering_angle, 
                                               imu_data.rotational_velocity);

  this->_has_made_prediction_ = true;
}


void NoRearWSSEKF::correct_wheels(Eigen::Vector3d& state, Eigen::Matrix3d& covariance,
                                  common_lib::sensor_data::WheelEncoderData& wss_data,
                                  double motor_rpm, double steering_angle) {
  if (std::abs(wss_data.fl_rpm - wss_data.fr_rpm) >= 60){
    return;
  }
  Eigen::VectorXd predicted_observations = this->s2v_model->cg_velocity_to_wheels(state);
  Eigen::VectorXd observations = Eigen::VectorXd::Zero(4);
  observations << wss_data.fl_rpm, wss_data.fr_rpm, steering_angle, motor_rpm;
  Eigen::VectorXd y = observations - predicted_observations;
  Eigen::MatrixXd jacobian = this->s2v_model->jacobian_cg_velocity_to_wheels(state);
  Eigen::MatrixXd kalman_gain =
      covariance * jacobian.transpose() *
      (jacobian * covariance * jacobian.transpose() + this->_wheels_measurement_noise_matrix_)
          .inverse();

  // DEBUG PRINTS
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Predicted observations: %f %f %f %f %f %f",
               predicted_observations(0), predicted_observations(1), predicted_observations(2),
               predicted_observations(3), predicted_observations(4), predicted_observations(5));
  RCLCPP_DEBUG(rclcpp::get_logger("velocity_estimation"),
               "correct_wheels - Observations: %f %f %f %f %f %f", observations(0), observations(1),
               observations(2), observations(3), observations(4), observations(5));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Covariance: \n"
                                                                     << covariance);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - y: \n" << y);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Jacobian: \n"
                                                                     << jacobian);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("velocity_estimation"), "correct_wheels - Kalman gain: \n"
                                                                     << kalman_gain);
  state += kalman_gain * y;
  if (this->_has_made_prediction_)
    covariance = (Eigen::Matrix3d::Identity() - kalman_gain * jacobian) * covariance;
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






Eigen::Matrix3d NoRearWSSEKF::compute_adaptive_process_noise(const Eigen::Matrix3d& base_noise_matrix,
                                                           double steering_angle) {
    Eigen::Matrix3d adaptive_noise = base_noise_matrix;
    
    double steering_noise_factor = 1.0 + 0.5 * std::abs(steering_angle);

    adaptive_noise(0, 0) *= steering_noise_factor;       // Longitudinal velocity noise
    adaptive_noise(1, 1) *= steering_noise_factor * 2.0; // Lateral velocity noise (higher)
    adaptive_noise(2, 2) *= steering_noise_factor;       // Angular velocity noise
    
    return adaptive_noise;
}


Eigen::Matrix3d NoRearWSSEKF::compute_steering_jacobian(
    const Eigen::Vector3d &previous_pose,
    const Eigen::Vector3d &accelerations,
    const double steering_angle,
    const double delta_t) {

  Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();

  const double v = accelerations(0);
  const double delta = steering_angle;
  const double theta = previous_pose(2);

  if (::abs(delta) < 1e-4) {
    jacobian(0, 0) = ::cos(theta) * delta_t;
    jacobian(1, 0) = ::sin(theta) * delta_t;
    jacobian(2, 0) = (delta / this->car_parameters_.wheelbase) * delta_t;
    jacobian(2, 2) = (v / this->car_parameters_.wheelbase) * delta_t;
  } else {
    const double L = this->car_parameters_.wheelbase;
    const double dt = delta_t;
    const double tan_delta = ::tan(delta);
    const double omega = (v / L) * tan_delta;
    const double theta_new = theta + omega * dt;

    // Partial derivatives with respect to velocity 'v'
    jacobian(0, 0) = ::cos(theta_new) * dt;
    jacobian(1, 0) = ::sin(theta_new) * dt;
    jacobian(2, 0) = (tan_delta / L) * dt;

    // Partial derivatives with respect to steering_angle 'delta'
    const double cos_delta_sq = ::cos(delta) * ::cos(delta);
    const double d_omega_d_delta = v / (L * cos_delta_sq);
    
    const double d_x_d_omega = (v / (omega * omega)) * (-::sin(theta_new) + omega * dt * ::cos(theta_new) + ::sin(theta));
    const double d_y_d_omega = (v / (omega * omega)) *
        (::cos(theta_new) + omega * dt * ::sin(theta_new) - ::cos(theta));
        
    jacobian(0, 2) = d_x_d_omega * d_omega_d_delta;
    jacobian(1, 2) = d_y_d_omega * d_omega_d_delta;
    jacobian(2, 2) = dt * d_omega_d_delta;
  }
  return jacobian;
}



Eigen::Vector3d NoRearWSSEKF::compute_steering_state_update(const Eigen::Vector3d& state,
                                                           const Eigen::Vector3d& accelerations,
                                                           double dt, double steering_angle,
                                                           double imu_angular_velocity) {
    double vx = state(0);
    double vy = state(1);
    double omega = state(2);

    double ax = accelerations(0);
    double ay = accelerations(1);

    Eigen::Vector3d next_accelerations;
    double v_magnitude = sqrt(vx*vx + vy*vy);
    
    // If steering angle is significant and vehicle is moving, use bicycle model
    if (std::abs(steering_angle) > 1e-4 && v_magnitude > 0.1) {

        next_accelerations(0) = vx + dt * (ax - vy * omega);
        next_accelerations(1) = vy + dt * (ay + vx * omega);

        double L = this->car_parameters_.wheelbase;
        next_accelerations(2) = (v_magnitude * tan(steering_angle)) / L;

    } 
    // if steering angle is negligible or vehicle is stationary, use simple kinematics
    else {
        return this->process_model->get_next_velocities(state, accelerations, dt);
    }
    
    return next_accelerations;
}

