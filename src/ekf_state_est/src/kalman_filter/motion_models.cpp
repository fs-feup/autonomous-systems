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

  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {
    double delta_x =
        motion_prediction_data.translational_velocity * cos(expected_state(2)) * time_interval;
    double delta_y =
        motion_prediction_data.translational_velocity * sin(expected_state(2)) * time_interval;

    next_state(0) += delta_x;
    next_state(1) += delta_y;
  } else {
    double radius =
        motion_prediction_data.translational_velocity / motion_prediction_data.rotational_velocity;
    double delta_x = -radius * sin(expected_state(2)) +
                     radius * sin(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);
    double delta_y = radius * cos(expected_state(2)) -
                     radius * cos(expected_state(2) +
                                  motion_prediction_data.rotational_velocity * time_interval);

    next_state(0) += delta_x;
    next_state(1) += delta_y;
  }

  next_state(3) = motion_prediction_data.translational_velocity;
  next_state(4) = motion_prediction_data.rotational_velocity;

  next_state(2) = common_lib::maths::normalize_angle(
      expected_state(2) + motion_prediction_data.rotational_velocity * time_interval);

  return next_state;
}

Eigen::MatrixXf NormalVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state, const MotionUpdate &motion_prediction_data,
    const double time_interval) const {
  Eigen::MatrixXf jacobian =
      Eigen::MatrixXf::Identity(expected_state.size(), expected_state.size());

  if (std::abs(motion_prediction_data.rotational_velocity) < 0.02) {
    jacobian(0, 2) = -motion_prediction_data.translational_velocity * std::sin(expected_state(2)) *
                     time_interval;
    jacobian(1, 2) =
        motion_prediction_data.translational_velocity * std::cos(expected_state(2)) * time_interval;
  } else {
    if (std::abs(motion_prediction_data.rotational_velocity) >
        std::numeric_limits<float>::epsilon()) {
      float ratio = motion_prediction_data.translational_velocity /
                    motion_prediction_data.rotational_velocity;
      float new_heading =
          expected_state(2) + motion_prediction_data.rotational_velocity * time_interval;

      jacobian(0, 2) = -ratio * (std::cos(expected_state(2)) - std::cos(new_heading));
      jacobian(1, 2) = -ratio * (std::sin(expected_state(2)) - std::sin(new_heading));
    }
  }

  jacobian(3, 2) = -motion_prediction_data.translational_velocity * std::sin(expected_state(2));
  jacobian(4, 2) = motion_prediction_data.translational_velocity * std::cos(expected_state(2));

  return jacobian;
}

/*----------------------IMU Velocity Model ------------------------*/

Eigen::VectorXf ImuVelocityModel::predict_expected_state(const Eigen::VectorXf &expected_state,
                                                         const MotionUpdate &motion_prediction_data,
                                                         const double time_interval) const {
  Eigen::VectorXf new_state = expected_state;

  if (std::abs(new_state(4)) < 0.0001) {  // Driving straight
    new_state(0) += new_state(3) * time_interval * std::cos(new_state(2));
    new_state(1) += new_state(3) * time_interval * std::sin(new_state(2));
    new_state(3) += new_state(5) * time_interval;
    new_state(4) = 0.0000001;
  } else {
    new_state(0) +=
        (new_state(3) / new_state(4)) *
        (std::sin(new_state(4) * time_interval + new_state(2)) - std::sin(new_state(2)));
    new_state(1) +=
        (new_state(3) / new_state(4)) *
        (-std::cos(new_state(4) * time_interval + new_state(2)) + std::cos(new_state(2)));
    new_state(2) =
        common_lib::maths::normalize_angle((new_state(2) + new_state(4) * time_interval));
    new_state(3) += new_state(5) * time_interval;
    new_state(4) = motion_prediction_data.rotational_velocity;
    new_state(5) = motion_prediction_data.acceleration;
  }

  return new_state;
}

Eigen::MatrixXf ImuVelocityModel::get_motion_to_state_matrix(
    const Eigen::VectorXf &expected_state,
    [[maybe_unused]] const MotionUpdate &motion_prediction_data, const double time_interval) const {
  float v = expected_state(3);
  float tr = expected_state(4);
  float theta = expected_state(2);
  float dt = time_interval;

  int state_size = expected_state.size();
  Eigen::MatrixXf JA = Eigen::MatrixXf::Identity(state_size, state_size);

  if (std::abs(tr) < 0.0001) {
    JA(0, 2) = -v * dt * std::sin(theta);
    JA(0, 3) = dt * std::cos(theta);
    JA(1, 2) = v * dt * std::cos(theta);
    JA(1, 3) = dt * std::sin(theta);
  } else {
    float a13 = (v / tr) * (std::cos(tr * dt + theta) - std::cos(theta));
    float a14 = (1.0 / tr) * (std::sin(tr * dt + theta) - std::sin(theta));
    float a15 = (dt * v / tr) * std::cos(tr * dt + theta) -
                (v / (tr * tr)) * (std::sin(tr * dt + theta) - std::sin(theta));
    float a23 = (v / tr) * (std::sin(tr * dt + theta) - std::sin(theta));
    float a24 = (1.0 / tr) * (-std::cos(tr * dt + theta) + std::cos(theta));
    float a25 = (dt * v / tr) * std::sin(tr * dt + theta) -
                (v / (tr * tr)) * (-std::cos(tr * dt + theta) + std::cos(theta));

    if (state_size >= 6) {
      JA(0, 2) = a13;
      JA(0, 3) = a14;
      JA(0, 4) = a15;
      JA(1, 2) = a23;
      JA(1, 3) = a24;
      JA(1, 4) = a25;
      JA(2, 4) = dt;
      JA(3, 5) = dt;
    }
  }

  return JA;
}