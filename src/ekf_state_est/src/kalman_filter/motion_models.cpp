#include "kalman_filter/motion_models.hpp"

#include "common_lib/maths/transformations.hpp"

MotionModel::MotionModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : _process_noise_covariance_matrix(process_noise_covariance_matrix) {}

NormalVelocityModel::NormalVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

ImuVelocityModel::ImuVelocityModel(const Eigen::MatrixXf &process_noise_covariance_matrix)
    : MotionModel(process_noise_covariance_matrix) {}

Eigen::MatrixXf MotionModel::get_process_noise_covariance_matrix(
    const unsigned int state_size) const {
  return Eigen::MatrixXf::Identity(state_size, 6) * this->_process_noise_covariance_matrix *
         Eigen::MatrixXf::Identity(6, state_size);
}

Eigen::MatrixXf MotionModel::create_process_noise_covariance_matrix(float process_noise) {
  return Eigen::MatrixXf::Identity(6, 6) * process_noise;
}

/*------------------------Normal Velocity Model-----------------------*/

Eigen::VectorXf NormalVelocityModel::predict_expected_state(
    const Eigen::VectorXf &expected_state, const MotionUpdate &motion_prediction_data,
    const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;
  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {  // Rectilinear movement

    next_state(0) +=
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
    next_state(1) +=
        motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    next_state(0) +=
        -(motion_prediction_data.translational_velocity / next_state(5)) * sin(expected_state(2)) +
        (motion_prediction_data.translational_velocity / next_state(5)) *
            sin(expected_state(2) + next_state(5) * time_interval);
    next_state(1) +=
        (motion_prediction_data.translational_velocity / next_state(5)) * cos(expected_state(2)) -
        (motion_prediction_data.translational_velocity / next_state(5)) *
            cos(expected_state(2) + next_state(5) * time_interval);
  }
  next_state(3) = motion_prediction_data.translational_velocity * cos(next_state(2));
  next_state(4) = motion_prediction_data.translational_velocity * sin(next_state(2));
  next_state(5) = motion_prediction_data.rotational_velocity;
  next_state(2) = common_lib::maths::normalize_angle(
      expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);

  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data,
    [[maybe_unused]] const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {  // Rectilinear movement
    jacobian(0, 2) =
        -motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;
    jacobian(1, 2) =
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
  } else {  // Curvilinear movement
    jacobian(0, 2) =
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            cos(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    // jacobian(0, 5) =
    //     ((motion_prediction_data.translational_velocity *
    //       motion_prediction_data.rotational_velocity *
    //       cos(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval))
    //+
    //      (motion_prediction_data.translational_velocity * sin(expected_state(2))) +
    //      motion_prediction_data.translational_velocity *
    //          sin(expected_state(2) + motion_prediction_data.rotational_velocity *
    // time_interval))
    //          /
    //     pow(motion_prediction_data.rotational_velocity, 2);
    jacobian(1, 2) =
        -(motion_prediction_data.translational_velocity /
          motion_prediction_data.rotational_velocity) *
            sin(expected_state(2)) +
        (motion_prediction_data.translational_velocity /
         motion_prediction_data.rotational_velocity) *
            sin(expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);
    // jacobian(1, 5) =
    //     -((motion_prediction_data.translational_velocity *
    //        motion_prediction_data.rotational_velocity *
    //        sin(expected_state(2) + motion_prediction_data.rotational_velocity *
    // time_interval)) -
    //       (motion_prediction_data.translational_velocity * cos(expected_state(2))) +
    //       motion_prediction_data.translational_velocity *
    //           cos(expected_state(2) + motion_prediction_data.rotational_velocity *
    //           time_interval)) /
    //     pow(motion_prediction_data.rotational_velocity, 2);
  }
  jacobian(3, 2) = -motion_prediction_data.translational_velocity * sin(expected_state(2));
  jacobian(4, 2) = motion_prediction_data.translational_velocity * cos(expected_state(2));
  return jacobian;
}

/*----------------------IMU Velocity Model ------------------------*/

Eigen::VectorXf ImuVelocityModel::predict_expected_state(const Eigen::VectorXf &expected_state,
                                                         const MotionUpdate &motion_prediction_data,
                                                         const double time_interval) const {
  Eigen::VectorXf next_state = expected_state;

  // next_state(0) += expected_state(3) * time_interval +
  //                  0.5 * motion_prediction_data.acceleration_x * pow(time_interval, 2);
  // next_state(1) += expected_state(4) * time_interval +
  //                  0.5 * motion_prediction_data.acceleration_y * pow(time_interval, 2);
  // next_state(5) = motion_prediction_data.rotational_velocity;
  // next_state(2) =
  //     common_lib::maths::normalize_angle(expected_state(2) + next_state(5) * time_interval);
  // next_state(3) += motion_prediction_data.acceleration_x * time_interval;
  // next_state(4) += motion_prediction_data.acceleration_y * time_interval;

  // return next_state;
  double theta = expected_state(2);
  double vx = expected_state(3);
  double vy = expected_state(4);
  double v = sqrt(vx * vx + vy * vy);
  double a = motion_prediction_data.acceleration_x;
  double omega = motion_prediction_data.rotational_velocity;

  // Handle division by zero (omega = 0 case)
  double omega_sq = omega * omega;
  double denom = (omega_sq != 0.0) ? omega_sq : 1.0;
  double omega_dt = omega * time_interval;

  double sin_omega_dt = sin(omega_dt);
  double cos_omega_dt = cos(omega_dt);
  double sin_theta = sin(theta);
  double cos_theta = cos(theta);

  double delta_x =
      (1.0 / denom) * ((v * omega + a * omega_dt) * sin(theta + omega_dt) +
                       a * cos(theta + omega_dt) - v * omega * sin_theta - a * cos_theta);
  double delta_y =
      (1.0 / denom) * ((-v * omega - a * omega_dt) * cos(theta + omega_dt) +
                       a * sin(theta + omega_dt) + v * omega * cos_theta - a * sin_theta);

  next_state(0) += delta_x;
  next_state(1) += delta_y;
  next_state(2) += omega_dt;
  double updated_v = v + a * time_interval;        // updated velocity magnitude
  next_state(3) = updated_v * cos(next_state(2));  // Vx
  next_state(4) = updated_v * sin(next_state(2));  // Vy
  return next_state;
}

Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data, const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  // double theta = expected_state(2);
  // double vx = expected_state(3);
  // double vy = expected_state(4);
  // double v = sqrt(vx * vx + vy * vy);
  // double a = motion_prediction_data.acceleration_x;
  // double o = motion_prediction_data.rotational_velocity;
  // double omega_sq = o * o;
  // double denom = (omega_sq != 0.0) ? omega_sq : 1.0;
  // jacobian(0, 2) = (cos(o * time_interval + expected_state(2)) * (v * o * time_interval) +
  //                   a * sin(theta) - a * sin(o * time_interval + theta) - v * o * cos(theta)) /
  //                  (denom);
  // jacobian(1, 2) = (a * cos(o * time_interval + theta) -
  //                   sin(o * time_interval + theta) * (-v * o - a * o * time_interval) -
  //                   v * o * sin(theta) - a * cos(theta)) /
  //                  (denom);

  // need to compute jacobian of the above code
  return jacobian;
}